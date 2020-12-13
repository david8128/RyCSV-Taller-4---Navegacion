#!/usr/bin/env python

import rospy
import roslib
import tf_conversions
import tf2_ros
import math
import numpy as np
import time
from geometry_msgs.msg import TransformStamped


class controller:

    def __init__(self):

        #Controller Gains
        self.k_p = 0.0
        self.k_a = 0.0
        self.k_b = 0.0  

        #Speed Limits
        self.cruise_lin = 0.0
        self.cruise_ang = 0.0

        #Goal Position
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.th_goal = 0.0

        #Goal transform broadcaster
        self.goal_br = tf2_ros.TransformBroadcaster()

        #Error transform listener
        self.tfBuffer = tf2_ros.Buffer()
        self.error_listener = tf2_ros.TransformListener(self.tfBuffer)

        #Errors (rectagular)
        self.error_x = 0.0
        self.error_y = 0.0
        self.error_th = 0.0

        #Errors (polar)
        self.alpha = 0.0
        self.beta = 0.0
        self.p = 0.0

        #Speeds (Controller outs)
        self.v_out = 0.0
        self.w_out = 0.0

        #Goal reached flag
        self.done = False

    def set_controller_params(self):
        #Get control parameters from ROS param server
        #This function is a callback from the dynamic reconfigure server
        self.k_p = rospy.get_param('/motion_control/k_p')
        self.k_a = rospy.get_param('/motion_control/k_a')
        self.k_b = rospy.get_param('/motion_control/k_b')
        self.cruise_lin = rospy.get_param('/motion_control/cruise_lin')
        self.cruise_ang = np.deg2rad(rospy.get_param('/motion_control/cruise_ang'))
        rospy.loginfo('*-- CONTROL PARAMS HAVE CHANGED --*')
        rospy.loginfo('p gain: '+str(self.k_p))
        rospy.loginfo('a gain: '+str(self.k_a))
        rospy.loginfo('b gain: '+str(self.k_b))
        rospy.loginfo('linear cruise speed: '+str(self.cruise_lin))
        rospy.loginfo('angular cruise speed: '+str(np.rad2deg(self.cruise_ang)))

    def broadcast_goal(self,now):
        #Broadcast goal position as TF transform

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom" #Fixed Frame
        t.child_frame_id = "goal" #Goal frame
        t.transform.translation.x = self.x_goal 
        t.transform.translation.y = self.y_goal
        t.transform.translation.z = 0.0
        #Transform from rpy (angles) to quaternion
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.th_goal)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.goal_br.sendTransform(t) #Broadcast
    
    def set_goal(self,x,y,th):
        #Set goal position and orientation (header frame: odom)
        #Orientation in degrees

        self.x_goal = x
        self.y_goal = y
        self.th_goal = np.deg2rad(th)
    
    def compute_error(self,now):
        #Measure error from base footprint to goal
        try:
            trans = self.tfBuffer.lookup_transform('goal', 'base_footprint', now, rospy.Duration(1.0))
            quat = np.zeros(4)
            quat[0] = trans.transform.rotation.x 
            quat[1] = trans.transform.rotation.y 
            quat[2] = trans.transform.rotation.z 
            quat[3] = trans.transform.rotation.w 
            #Transform from quaternion to rpy
            rpy = tf_conversions.transformations.euler_from_quaternion(quat)
            self.error_x = trans.transform.translation.x * -1 #Negative (measured from goal to base, inverted)
            self.error_y = trans.transform.translation.y * -1
            self.error_th = rpy[2] #Orientation respecting right hand rule

            #Print error for debugging
            print("--")
            print("Rectangular Errors")
            print("X:"+str(self.error_x))
            print("Y:"+str(self.error_y))
            print("Theta: "+str(self.error_th)+" rad / "+str(np.rad2deg(self.error_th))+" deg")
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Exception")
            

    def transform_error(self):
        #Transform error from X-Y cartesian cordinates to polar coordinates
        #Refer to: Roland Siegwart, Intro to autonomus mobile robots - Chap 3 (Control Law)
        self.p = math.sqrt((pow(self.error_x,2) + pow(self.error_y,2)))
        self.alpha = (-1 * self.error_th) + math.atan2(self.error_y,self.error_x)
        self.beta = (-1 * self.error_th) - (self.alpha)

        #Print polar error for debugging
        print("--")
        print("Polar Errors")
        print("Alpha: "+str(self.alpha)+" rad / "+str(np.rad2deg(self.alpha))+" deg")
        print("Beta: "+str(self.beta)+" rad / "+str(np.rad2deg(self.beta))+" deg")
        print("Rho: "+str(self.p))
        
    
    def control_speed(self):

        #Control Law
        #Refer to: Roland Siegwart, Intro to autonomus mobile robots - Chap 3 (Control Law)
        if(self.error_x<0.02 and self.error_y<0.02):
            v = 0                                               #Linear speed
            w = -self.k_b * self.beta                      #Angular speed
        else:
            v = self.p * self.k_p                               #Linear speed
            w = self.k_a * self.alpha + self.k_b * self.beta    #Angular speed

        self.v_out = v
        self.w_out = w

        #Limit Speed (Saturation)
        if np.abs(v) > self.cruise_lin: 
            self.v_out = np.sign(v) * self.cruise_lin
        if np.abs(w) > self.cruise_ang:
            self.w_out = np.sign(w) * self.cruise_ang

        #Reorient no linear move
        if (np.abs(self.error_x) < 0.02 and np.abs(self.error_y) < 0.02):
            self.v_out = 0

        #Print controller out for debugging
        print("--")
        print("Controller Out")
        print("V out :"+str(self.v_out))
        print("W out :"+str(self.w_out))

        
    def check_goal_reached(self):
        if (np.abs(self.error_x) < 0.02 and np.abs(self.error_y) < 0.02 and np.abs(self.error_th) < np.deg2rad(2)):
            self.done = True
            self.v_out = 0
            self.w_out = 0
        else:
            self.done = False
        