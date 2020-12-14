#!/usr/bin/env python

import rospy
import time
import numpy as np
from controlador import controller
from dynamic_reconfigure.server import Server
from rycsv_kobuki_localization.cfg import controllerConfig
from geometry_msgs.msg import Twist

def angle_between(p0,p1,p2):
    v0 = np.array(p1) - np.array(p0)
    v1 = np.array(p2) - np.array(p0)

    angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
    return angle

def xy2traj(dots):
    """
    From xy coordinates generates a complete trajectory

    Takes x,y coordinates of a trajectory and calculates x,y,theta coordinates
    with intermidiate points that assure twists of 90
    
    Parameters
    ----------
    dots : list of [x,y]
        Dots [[x_0,y_0],[x_1,y_1],...]
    Motion : control.Motion
        Class where the control and motion is settled

    Returns
    -------
    list of [x,y,theta]
        Complete trajectory

    """
    traj = []
    last_dot = 0 
    last_x = 0
    last_y = 0
    for count, dot in enumerate(dots):
        x = dot[0]
        y = dot[1]
        if (count == 0) :
            theta = 90
            traj.append([x,y,theta])           #Radians to deg
        else:
            theta = angle_between(last_dot,[last_x+1,last_y],dot)
            traj.append([last_x,last_y,np.rad2deg(theta)])
            traj.append([x,y,np.rad2deg(theta)])
        last_dot = dot
        last_theta = theta
        last_x = x
        last_y = y
    return traj

#Callback function from dynamic reconfigure
def callback(config, level):
    global dyn_flag
    dyn_flag = 1
    print('Param change requested...')
    return config
    

if __name__ == "__main__":

    #Node initialization
    rospy.init_node("motion_control", anonymous = False)
    rate = rospy.Rate(50) # 50 Hz ROS

    #Controller init
    kobuki_controller = controller()

    #Dynamic reconfigure flag
    dyn_flag = 0

    #Dynamic reconfigure server initialization
    srv = Server(controllerConfig, callback)

    #Wheel speed publisher
    nameSpeedTopic = "/mobile_base/commands/velocity"
    kobuki_speed_pub = rospy.Publisher(nameSpeedTopic, Twist, queue_size=10)
    command = Twist()

    #Trajectory dots (No orientation)
    dots = [
            [0, 0], [-3.5, 0], [-3.5, 3.5], [1.5, 3.5],
            [1.5, -1.5], [3.5, -1.5], [3.5, -8.0], 
            [-2.5, -8.0], [-2.5, -5.5], [1.5, -5.5], 
            [1.5, -3.5], [-1.0, -3.5]
        ]

    #Add orientation to trajectory
    traj = xy2traj(dots)
    traj = np.array(traj)
    #Trajectory limits
    goal_id = 0
    dot_count, coord = traj.shape

    #Initial point
    kobuki_controller.set_goal(traj[goal_id][0],traj[goal_id][1],traj[goal_id][2])
    print("--")
    print("GOAL")
    print("X: "+str(kobuki_controller.x_goal))
    print("Y: "+str(kobuki_controller.y_goal))
    print("Z: "+str(kobuki_controller.th_goal))
    print("--")
    #Node Loop
    while(not rospy.is_shutdown()):
        
        #Check and change controllers params if requested 
        if dyn_flag == 1:
            kobuki_controller.set_controller_params()
            dyn_flag = 0




        #Get "now" time to syncronize target tf and error tf 
        now = rospy.Time.now()

        #Broadcast goal TF
        kobuki_controller.broadcast_goal(now)

        #Control methods
        kobuki_controller.compute_error(now)
        kobuki_controller.transform_error()
        kobuki_controller.control_speed()

        #Publish Speed
        command.linear.x =  kobuki_controller.v_out
        command.angular.z =  kobuki_controller.w_out
        kobuki_speed_pub.publish(command)

        #Check if goal has been reached
        kobuki_controller.check_goal_reached()
        if kobuki_controller.done:  
            print("Goal has been reached...")
            print("--")
            goal_id = goal_id+1      #Change point when arrived to goal

            if goal_id == dot_count:
                goal_id = goal_id-1         #Wait at last point
                #goal_id = 0                 #Go back to initial point

            kobuki_controller.set_goal(traj[goal_id][0],traj[goal_id][1],traj[goal_id][2])
            now = rospy.Time.now()
            kobuki_controller.broadcast_goal(now) 
            kobuki_controller.compute_error(now)

        #Check and change controllers params if requested 
        if dyn_flag == 1:
            kobuki_controller.set_controller_params()
            dyn_flag = 0
        
        rate.sleep() #Wait for ROS node cycle
            



