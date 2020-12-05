#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from hector_uav_msgs.srv import EnableMotors
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import time
#import tf
import math
from _ast import IsNot
from amcl.cfg.AMCLConfig import inf
import numpy as np

def wraptopi(number):
    return (number + np.pi) % (2 * np.pi) - np.pi


class WallFollower:

    ref_distance_from_wall = 0.5
    max_speed = 0.2
    altitude = 0.5
    front_range = 0.0
    side_range = 0.0
    max_rate = 0.5
    state_start_time = 0
    state = "FORWARD"
    previous_heading = 0.0
    sizeOfRoom = 3
    distanceToGoAwayFromWall = sizeOfRoom/2
    angle = 2000
    calculate_angle_first_time = True
    around_corner_first_turn = True
    direction = 1
    around_corner_go_back = False

    def init(self, new_ref_distance_from_wall, max_speed_ref = 0.2):
        self.ref_distance_from_wall = new_ref_distance_from_wall
        self.state = "TURN_TO_FIND_WALL"
        self.max_speed = max_speed_ref

    def take_off(self):
        twist = Twist()
        twist.linear.z = 0.1
        return twist

    def hover(self):
        twist = Twist()
        return twist

    def twistForward(self):
        v = self.max_speed
        w = 0
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist

    def twistRight(self):
        v = self.max_speed
        w = 0
        twist = Twist()
        # TODO verify the direction, don't know if - should be for right or left
        twist.linear.y = v
        twist.angular.z = w
        return twist

    def twistLeft(self):
        v = self.max_speed
        w = 0
        twist = Twist()
        # TODO verify the direction, don't know if - should be for right or left
        twist.linear.y = -v
        twist.angular.z = w
        return twist

    def twistForwardAlongWall(self, range):
        twist = Twist()
        # max speed forward
        twist.linear.x = self.max_speed
        # check if the drone is close to an obstacle
        if not self.logicIsCloseTo(self.ref_distance_from_wall, range, 0.1):
            if range > self.ref_distance_from_wall:
                # if its further away to the wall than the reference desired distance, it changes
                # its Y value to get closer to it
                twist.linear.y = self.direction * (- self.max_speed/3)
            else:
                twist.linear.y = self.direction*self.max_speed/3
        return twist

    def twistTurn(self, rate):
        # no linear velocity, just stays in place and yaw
        v = 0.0
        # angular speed
        w = self.direction*rate
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist

    def twistTurnandAdjust(self, rate, range):
        # no linear velocity, just stays in place and yaw
        v = 0.0
        # angular speed
        w = self.direction*rate
        twist = Twist()

        if not self.logicIsCloseTo(self.ref_distance_from_wall, range, 0.1):
            if range > self.ref_distance_from_wall:
                twist.linear.y = self.direction*(- self.max_speed/3)
            else:
                twist.linear.y = self.direction*(self.max_speed/3)
        twist.linear.x = v
        twist.angular.z = w
        return twist

    def twistTurnAroundCorner(self, radius, bias):
        v = self.max_speed
        # angular speed
        w = self.direction*(-v/radius)+bias
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist

    # to check if the drone is close to an obstacle
    def logicIsCloseTo(self, real_value = 0.0, checked_value =0.0, margin=0.05):
        if (real_value > checked_value - margin) and (real_value < checked_value + margin):
            return True
        else:
            return False

    # Transition state and restart the timer
    def transition(self, newState):
        state = newState
        self.state_start_time = time.time()
        return state

    def wall_follower(self, front_range, side_range, current_heading, direction_turn=1):
        self.direction = direction_turn

        ##### handle state transitions #######
        if self.state == "TAKE_OFF":
            # if higher than 0.5, fly forward
            if self.altitude > 0.5:
                self.state = self.transition("FORWARD")

        elif self.state == "FORWARD":
            # if close enough to the front wall, turn to find it and be in front of it.
            if front_range < self.ref_distance_from_wall:
                self.state = self.transition("TURN_TO_FIND_WALL")

        elif self.state == "HOVER":
            print(self.state)

        elif self.state == "TURN_TO_FIND_WALL":
            print("ranges (front, side): ", front_range, side_range)
            # if in front of the wall
            if (side_range < self.ref_distance_from_wall/math.cos(0.78)+0.2 and front_range < self.ref_distance_from_wall/math.cos(0.78)+0.2):
                self.previous_heading = current_heading
                self.angle = self.direction*(1.57 - math.atan(front_range/side_range))
                # time to rotate in order to be parallel to the wall
                self.state = self.transition("TURN_TO_ALLIGN_TO_WALL")
                print("got angle ", self.angle)
            # if there is no corner closer than 2m and we're next to a wall, it means we should rotate around the wall
            # careful to the 2.0 value, we might want to change this value depending on the size of th walls (room)
            if (side_range < 1.0 and front_range > 2.0):
                self.around_corner_first_turn = True
                self.around_corner_go_back = False
                self.previous_heading = current_heading
                self.state = self.transition("ROTATE_AROUND_WALL")

        elif self.state =="TURN_TO_ALLIGN_TO_WALL":
            print(current_heading)
            print(wraptopi(current_heading-self.previous_heading), self.angle)
            if self.logicIsCloseTo(wraptopi(current_heading-self.previous_heading), self.angle, 0.1):
                self.state = self.transition("FORWARD_ALONG_WALL")

        elif self.state == "FORWARD_ALONG_WALL":
            # if the drone is too far from the wall, it means that there was an opening or a corner
            # so it should turn there to explore it.
            if side_range > 2:
                self.around_corner_first_turn = True
                self.state = self.transition("ROTATE_AROUND_WALL")
            # if it gets close to a wall in front of it, it means that it reached a corner and it has to rotate to
            # follow the next wall
            if front_range < self.ref_distance_from_wall:
                self.state = self.transition("ROTATE_IN_CORNER")
                self.previous_heading = current_heading
            # if 3 sec have been elapsed, then it will explore the center area for a it and then come back to the wall.
            if self.state_start_time > 3:
                self.state = self.transition("GET_AWAY_FROM_WALL")

        # fly perpendicular to the wall to explore a bit the center of the room while not too far from the wall
        elif self.state == "GET_AWAY_FROM_WALL":
            if side_range >= self.distanceToGoAwayFromWall:
                # goes back to the wall to continue the wall following
                self.state = self.transition("GO_BACK_TO_WALL")

        # goes back to the wall to continue the wall following
        elif self.state == "GO_BACK_TO_WALL":
            if side_range <= self.ref_distance_from_wall + 0.2:
                # if close enough to the wall again, then forward along wall again
                self.state = self.transition("FORWARD_ALONG_WALL")

        elif self.state =="ROTATE_AROUND_WALL":
            if front_range < self.ref_distance_from_wall+0.3:
                # If it's close from a wall (front), then it's time to rotate and place itself in front of the wall.
                self.state = self.transition("TURN_TO_FIND_WALL")

        elif self.state == "ROTATE_IN_CORNER":
            print(current_heading-self.previous_heading)
            if self.logicIsCloseTo(math.fabs(wraptopi(current_heading-self.previous_heading)), 0.8, 0.1):
                self.state = self.transition("TURN_TO_FIND_WALL")

        print(self.state)

        ##### handle state ations ########
        if self.state == "TAKE_OFF":
            twist = self.take_off()

        elif self.state == "FORWARD":
            twist = self.twistForward()

        elif self.state == "HOVER":

            twist = self.hover()
        elif self.state == "TURN_TO_FIND_WALL":
            twist = self.hover()
            if (time.time() - self.state_start_time) > 1:
                twist = self.twistTurn(self.max_rate)

        elif self.state == "TURN_TO_ALLIGN_TO_WALL":
            # hover for 2 sec, perform the calculus and then turn
            twist = self.hover()
            if (time.time() - self.state_start_time) > 2:
                twist = self.twistTurn(self.max_rate)

        elif self.state == "FORWARD_ALONG_WALL":
            twist = self.twistForwardAlongWall(side_range)

        # TODO test
        elif self.state == "GO_BACK_TO_WALL":
            twist = self.twistLeft()

        # TODO test
        elif self.state == "FORWARD_ALONG_WALL":
            twist = self.twistRight()

        elif self.state == "ROTATE_AROUND_WALL":
            if self.around_corner_first_turn:
                print("regular_turn_first")
            #if side_range>self.ref_distance_from_wall+0.5 and self.around_corner_first_turn:
                twist = self.twistTurn(-self.max_rate)
                if side_range <= self.ref_distance_from_wall+0.5:
                    self.around_corner_first_turn = False
                    self.previous_heading = current_heading
            else:
                if side_range > self.ref_distance_from_wall+0.5:
                    print("twistTurnandAdjust")
               # twist = self.twistTurnandAdjust(self.max_rate,side_range)
                    if wraptopi(abs(current_heading - self.previous_heading)) > 0.3:
                        self.around_corner_go_back = True
                    if self.around_corner_go_back:
                        twist = self.twistTurnandAdjust(self.max_rate, side_range)
                        print("go back")
                    else:
                        twist = self.twistTurnandAdjust(-1*self.max_rate, side_range)
                        print("forward")
                else:
                    print("twistTurnAroundCorner")
                    self.previous_heading = current_heading;
                    if self.around_corner_go_back:
                        twist = self.twistTurnAroundCorner(self.ref_distance_from_wall+0.2, 0.05)
                    else:
                        twist = self.twistTurnAroundCorner(self.ref_distance_from_wall+0.2, -0.05)
                    self.previous_heading = current_heading
                    self.around_corner_go_back = False

        elif self.state == "ROTATE_IN_CORNER":
            twist = self.twistTurn(self.max_rate)

        return twist, self.state


# Main method
if __name__ == '__main__':
    try:
        # TODO fix this error (next line)
        wall_follower()
    except rospy.ROSInterruptException:
        pass
