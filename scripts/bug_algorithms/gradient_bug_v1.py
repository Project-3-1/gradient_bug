#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from hector_uav_msgs.srv import EnableMotors
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from wall_follower_multi_ranger import WallFollower

#import matplotlib.pyplot as plt
from sklearn import linear_model, datasets
import matplotlib.pyplot as plt
from scipy.signal import medfilt


import time
#import tf
import math
from _ast import IsNot

import numpy as np

from amcl.cfg.AMCLConfig import inf

def wraptopi(number):
    return  ( number + np.pi) % (2 * np.pi ) - np.pi


class GradientBugController:
    wall_follower = WallFollower()
    ref_distance_from_wall = 1.0
    max_speed = 0.2
    front_range = 0.0
    right_range = 0.0
    max_rate = 0.5
    state_start_time = 0
    state = "FORWARD"
    previous_heading = 0.0;
    angle=2000
    calculate_angle_first_time = True;
    around_corner_first_turn = True;
    heading_prev = 0.0
    heading = 0.0
    first_scan = True
    scan_obstacle_done = False
    scan_obstacle_array = []
    scan_angle_array = []
    wall_angle = 0
    first_run = True
    prev_distance = 1000.0
    already_reversed_direction=False
    state_WF =  ""
    direction = 1
   # rssi_array =  np.zeros(360)

    '''plt.ion()
    fig = plt.figure()
    ax = plt.subplot(111)#, projection='polar')
    it_plot = 0'''
    
    rssi_tower_prev = -100
    
    do_circle = False
    
    angle_rssi = 2000
    
    
    rssi_array=[]
    rssi_heading_array =[]
    
    rssi_goal_angle_adjust = 0;

    '''
    def plotHeadingRSSI(self,rssi_value):
        #self.ax.plot(self.heading,-1.0*rssi_value,'ro')
        heading_array = range(-180,180)
        #self.ax.plot(time.time(),-1.0*rssi_value,'ro')
        
        self.ax.plot(heading_array,self.rssi_array,'ro')

        self.ax.grid(True)

        #plt.hold(True)

        self.fig.canvas.draw()

     '''
    def init(self,new_ref_distance_from_wall,max_speed_ref = 0.2):
        self.ref_distance_from_wall = new_ref_distance_from_wall
        self.state = "ROTATE_TO_GOAL"
        self.max_speed = max_speed_ref
        self.first_run = True
        #self.rssi_array =  np.zeros(360)


    def take_off(self):
        twist = Twist()
        twist.linear.z = 0.1;
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
    
    def twistTurn(self, rate):
        v = 0
        w = rate
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    def twistTurnCircle(self, radius):
        v = self.max_speed
        w = self.direction*(-v/radius)
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    
    def twistForwardAlongWall(self, range):
        twist = Twist()
        twist.linear.x = self.max_speed
        if  self.logicIsCloseTo(self.ref_distance_from_wall, range, 0.1) == False:
            if range>self.ref_distance_from_wall:
                twist.linear.y = self.direction*( - self.max_speed/3)
            else:
                twist.linear.y = self.direction*self.max_speed /3

        return twist
    
        
    def calculateWallRANSAC(self, range_ptx,angle_pty, scan_angle):
        
        points_ptx =  np.zeros(len(angle_pty))
        points_pty =  np.zeros(len(angle_pty))
        
        for it in range(0,len(angle_pty)):
            points_ptx[it] = math.sin(angle_pty[it])*range_ptx[it]
            points_pty[it] = math.cos(angle_pty[it])*range_ptx[it]
            if np.isnan(points_ptx[it]) or np.isinf(points_ptx[it]):
                points_ptx[it] = 0
            if np.isnan(points_pty[it]) or np.isinf(points_pty[it]):
                points_pty[it] = 0
            

        
        ransac = linear_model.RANSACRegressor()
        ransac.fit(points_ptx.reshape(-1,1), points_pty.reshape(-1,1) )
        inlier_mask = ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        line_y_ransac = ransac.predict(points_ptx.reshape(-1,1))

        wall_angle = ransac.estimator_.coef_[0][0]
        
        #plt.plot(points_ptx,points_pty)
       # plt.hold(True)
        #plt.plot(points_ptx,line_y_ransac);  
        #plt.ylim(0,1.1)            
        #plt.show()
        #time.sleep(10)
        
        return wall_angle
    '''

    def rssiFillArray(self,heading,rssi_value):
        heading_deg = int(np.rad2deg(heading))
        self.rssi_array[heading_deg] = rssi_value
       ''' 



    def logicIsCloseTo(self,real_value = 0.0, checked_value =0.0, margin=0.05):

        if real_value> checked_value-margin and real_value< checked_value+margin:
            return True
        else:
            return False

    # Transition state and restart the timer
    def transition(self, newState):
        state = newState
        self.state_start_time = self.simulator_time
        return state
    
    


        

    def stateMachine(self, front_range, right_range, left_range, current_heading, angle_goal, distance_goal, rssi_to_tower, correct_time, from_gazebo = True, WF_argos = None, RRT= None):
        twist = Twist()
        self.simulator_time = correct_time;
        print("time", correct_time)

        if front_range == None:
            front_range = inf

        if right_range == None:
            right_range = inf
            
        if left_range == None:
            left_range = inf

        
        '''
        if self.it_plot>10:
            self.plotHeadingRSSI(rssi_to_tower)
            self.it_plot = 0
        else:
            self.it_plot=self.it_plot+1
        '''
        
        
        if self.first_run is True:
            self.prev_distance = distance_goal;
            self.angle_rssi = angle_goal
            self.first_run = False

        
        '''
        if self.rssi_tower_prev - rssi_to_tower < 0:
            print("going right direction")
        elif self.rssi_tower_prev - rssi_to_tower == 0:
            print("going right direction with no change")
        elif self.rssi_tower_prev - rssi_to_tower > 0:
            print("going wrong direction!!")
            '''
        #print("current_heading before", current_heading)
        #current_heading = wraptopi(current_heading - self.rssi_goal_angle_adjust)
        #print("current_heading before", current_heading)
        bearing = angle_goal;#wraptopi(angle_goal-current_heading)

        bearing_with_adjust = angle_goal;# wraptopi(angle_goal-self.rssi_goal_angle_adjust)
        self.heading = current_heading;
        print("self.rssi_goal_angle_adjust ",self.rssi_goal_angle_adjust)
        print("self.current_heading ",current_heading)
        print("self.angle_rssi",self.angle_rssi)
        print("self.angle_goal",angle_goal)

        #print("angle_bearing", bearing)
        #print("angle_bearing with adjust", bearing_with_adjust)
        
       # bearing_with_adjust = bearing

        #print("rssi adjust angle", self.rssi_goal_angle_adjust)
        # Handle State transition
        if self.state == "FORWARD":
            if self.do_circle == True and correct_time-self.state_start_time > 1:
                rssi_array = []
                rssi_heading_array = []
                self.heading_prev=current_heading
                self.state = self.transition("ROTATE_180")
            if front_range < self.ref_distance_from_wall+0.2:
                self.state = self.transition("WALL_FOLLOWING")
                if from_gazebo:
                    self.wall_follower.init(self.ref_distance_from_wall,self.max_speed)
                else:
                    WF_argos.init()

                
                self.heading_prev = self.heading
                #self.first_scan = True
                #self.scan_obstacle_done = False
                #self.scan_obstacle_array = []
                #self.scan_angle_array = []
                self.already_reversed_direction = False
                if left_range<right_range and left_range < 2.0:
                    self.direction = -1
                if left_range>right_range and right_range < 2.0:
                    self.direction = 1
                if left_range>2.0 and right_range>2.0:
                    self.direction = 1



                
            '''
            if right_range < self.ref_distance_from_wall+0.2:
                self.direction = 1
                self.state = self.transition("WALL_FOLLOWING")
                self.wall_follower.init(self.ref_distance_from_wall,self.max_speed)
                self.heading_prev = self.heading
                            '''


        elif self.state == "HOVER":
            if   time.time()-self.state_start_time>1:
                self.state = self.transition("SCAN_OBSTACLE")
        elif self.state =="SCAN_OBSTACLE":
            if self.scan_obstacle_done is True:
                self.wall_angle=self.calculateWallRANSAC(self.scan_obstacle_array,self.scan_angle_array,0.52)
                self.state = "WALL_FOLLOWING"
        elif self.state =="REVERSE_DIRECTION":
            if front_range < self.ref_distance_from_wall+0.2:
                self.state = self.transition("WALL_FOLLOWING")
                if self.direction == -1:
                    self.wall_angle = -1
                elif self.direction == 1:
                    self.wall_angle = 1
        elif self.state == "WALL_FOLLOWING":
            if self.state_WF is "ROTATE_AROUND_WALL" or self.state_WF is "ROTATE_AROUND_CORNER":
                if front_range>1.5 and (bearing_with_adjust>-0.2 and bearing_with_adjust < 0.2):
                    self.state = self.transition("ROTATE_TO_GOAL")
                    self.do_circle = True

                    

            if self.prev_distance<distance_goal and self.already_reversed_direction is False:
                self.state = self.transition("REVERSE_DIRECTION")
                self.already_reversed_direction = True

        elif self.state=="ROTATE_TO_GOAL":            
            if self.logicIsCloseTo(bearing_with_adjust,0,0.1):
                self.state = self.transition("FORWARD")

        elif self.state=="ROTATE_180":
            if correct_time-self.state_start_time > 2 and self.logicIsCloseTo(current_heading,wraptopi( self.heading_prev),0.1) and self.do_circle:
                self.do_circle = False
                print(self.rssi_array)
                print(self.rssi_heading_array)
                rssi_array_filt = medfilt(self.rssi_array,9)
                                
                                
                print(self.rssi_array)
                index_max_rssi =np.argmax(rssi_array_filt)
                print(index_max_rssi)
                self.angle_rssi =wraptopi(self.rssi_heading_array[index_max_rssi]+3.14)
                self.state = self.transition("ROTATE_TO_GOAL")
                self.rssi_goal_angle_adjust = self.angle_rssi#wraptopi(current_heading-self.angle_rssi)
        


        

        # Handle actions
        if self.state == "FORWARD":
            twist=self.twistForward()
            if(left_range<self.ref_distance_from_wall):
                twist.linear.y = twist.linear.y-0.2;
            if(right_range<self.ref_distance_from_wall):
                twist.linear.y = twist.linear.y+0.2;
            
        if self.state == "HOVER":
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
        if self.state =="SCAN_OBSTACLE":
            scan_angle = 0.52;
            if self.first_scan is True:
                twist=self.twistTurn(-0.5)
                if  self.logicIsCloseTo(wraptopi(self.heading_prev - scan_angle),current_heading,0.1):
                    self.first_scan = False
            else:
                self.scan_obstacle_array.append(front_range)
                self.scan_angle_array.append(wraptopi(self.heading_prev - current_heading))
                
                twist=self.twistTurn(0.5)
                if self.logicIsCloseTo(wraptopi(self.heading_prev + scan_angle),current_heading,0.1):
                    self.scan_obstacle_done = True
        if self.state =="REVERSE_DIRECTION":
            twist = self.twistTurn(self.direction*-0.5)
        elif self.state == "WALL_FOLLOWING":
            #if self.wall_angle <= 0:
            if from_gazebo is True:
                if self.direction is 1:
                    twist, self.state_WF = self.wall_follower.wall_follower(front_range,right_range, current_heading,self.direction)
                else:
                    twist, self.state_WF = self.wall_follower.wall_follower(front_range,left_range, current_heading,self.direction)
            else:
                if self.direction is 1:
                    twist, self.state_WF = WF_argos.wallFollowingController(RRT.getRangeRight(),RRT.getRangeFrontRight(), RRT.getLowestValue(),RRT.getHeading(),RRT.getArgosTime(),self.direction)
                else:
                    twist, self.state_WF = WF_argos.wallFollowingController(RRT.getRangeLeft(),RRT.getRangeFrontLeft(), RRT.getLowestValue(),RRT.getHeading(),RRT.getArgosTime(),self.direction)
                
                twist.angular.z = twist.angular.z*-1

                print("ranges", RRT.getRangeFrontRight(),RRT.getRangeRight(),RRT.getRangeFrontLeft(),RRT.getRangeLeft())
        elif self.state=="ROTATE_TO_GOAL":
            
            if bearing_with_adjust>0:
                twist = self.twistTurn(self.max_rate)
            else:
                twist = self.twistTurn(-1*self.max_rate)
        elif self.state =="ROTATE_180":
            if self.do_circle is True:
                self.rssi_array.append(rssi_to_tower)
                self.rssi_heading_array.append(angle_goal)
                

            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.angular.z = self.max_rate;
            
            

        print(self.state)


        self.lastTwist = twist
        
        
        
        '''
        
        if(self.state =="FORWARD" or self.state_WF =="FORWARD_ALONG_WALL"):
            self.rssi_array = np.zeros(360)
        
        
        if(self.state_WF == "ROTATE_IN_CORNER" or self.state_WF == "ROTATE_AROUND_WALL" or self.state =="ROTATE_TO_GOAL"):
            self.rssiFillArray(current_heading,rssi_to_tower)
            self.plotHeadingRSSI(rssi_to_tower)
            
            
            '''
        
        self.rssi_tower_prev = rssi_to_tower
        
        return twist, self.rssi_goal_angle_adjust



if __name__ == '__main__':
    try:
        wall_follower()
    except rospy.ROSInterruptException:
        pass