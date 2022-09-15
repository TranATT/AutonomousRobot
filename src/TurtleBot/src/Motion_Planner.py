#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from gazebo_msgs.msg import ModelStates

from std_msgs.msg import Float64MultiArray
import tf
from math import radians, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import os
import time

goal_x = 0
goal_y = 0
goal_angle = 0
mode = -1

position_x = 0
position_y = 0
rotation = 0

cmd_vel = None
rrt_out = None
move_cmd = Float64MultiArray()

target_x = 0
target_y = 0

traj = []

def move(input_x, input_y, input_angle, input_mode):
    move_cmd.data = [input_x, input_y, input_angle, input_mode]
    cmd_vel.publish(move_cmd)
def pose_update(data):
    global position_x
    global position__y
    global rotation
    position_x = data.pose[1].position.x
    position_y = data.pose[1].position.y
    rotation = data.pose[1].orientation.z

def trajectory_update(data):
    global traj_arr
    traj_arr = data.data
    global traj
    for i in range(0, len(traj_arr), 2):
        output_temp = [traj_arr[i], traj_arr[i+1]]
        traj.append(output_temp)

    print(traj)

def target_update(data):
    global target
    global target_x
    global target_y
    target = data.data
    target_x = target[0]
    target_y = target[1]
    print(target)

if __name__ == '__main__':
    global goal_x
    global goal_y
    global position_x
    global position_y
    global goal_angle
    global mode
    global rotation
    
    try:
        rospy.init_node('Motion_Planner', anonymous=False)
    	cmd_vel = rospy.Publisher('/reference_pose', Float64MultiArray, queue_size=5)
    	rospy.Subscriber('/gazebo/model_states', ModelStates, pose_update)
    
    	rrt_out = rospy.Publisher('/start_goal', Float64MultiArray, queue_size = 10)
        rospy.Subscriber('/trajectory', Float64MultiArray, trajectory_update)

        rospy.Subscriber('/target_pose', Float64MultiArray, target_update)


    
    	r = rospy.Rate(10)

    	inp = input("0 for PID or 1 for RRT:")
    
        if (inp == 0):
            input_x = input('Enter goal X:')
            input_y = input('Enter goal Y:')
	    input_angle = input('Enter goal angle:')
    	    input_mode = input('Enter mode:')

    	    move(input_x, input_y, input_angle, input_mode)
    	    goal_x = input_x
    	    goal_y = input_y
    	    goal_angle = input_angle
    	    mode = input_mode

    	    while(True):
                error = sqrt(pow(goal_x - position_x, 2) + pow(goal_y - position_y, 2))
                if (error > 0.05):
            	    pass
                if (error < 0.05):
                    print('Goal Reached!')
                    input_x = input('Enter new goal X:')
                    input_y = input('Enter new goal Y:')
                    input_angle = input('Enter new goal angle:')
                    input_mode = input('Enter mode:')

                    move(input_x, input_y, input_angle, input_mode)
                    goal_x = input_x
                    goal_y = input_y
                    goal_angle = input_angle
                    mode = input_mode
        elif(inp == 1):
        
            output = Float64MultiArray()
            x_start_r = float(input('Enter x_start_r:'))
            y_start_r = float(input('Enter y_start_r:'))
            x_goal_r = float(input('Enter x_goal_r:'))
            y_goal_r = float(input('Enter y_goal_r:'))

            out = [x_start_r, y_start_r, x_goal_r, y_goal_r]
        
            output.data = out

            rrt_out.publish(output)
        else:
            x_start_r = position_x
            y_start_r = position_y
            while (target_x == 0 and target_y == 0):
                continue    
            x_goal_r = target_x
            y_goal_r = target_y
            out = [x_start_r, y_start_r, x_goal_r, y_goal_r]
            print(out) 
            output = Float64MultiArray()
            output.data = out

            rrt_out.publish(output)


            i = 0
            #cmd_vel.publish(traj[0])
            while (len(traj) == 0):
                continue
            print(traj)
            temp2 = traj[0]
            tempX = temp2[0]
            tempY = temp2[1]
            angle = 3.14
            move(tempX, tempY, angle , 1)
            print('Move 1')
            while(True):
                if (abs(target_x - position_x) <= 0.05 and abs(target_y - position_y)):
                    i = i+1
                    temp2 = traj[i]
                    tempX = temp2[0]
                    tempY = temp2[1]
                    angle = 3.14
                    move(tempX, tempY, angle , 1)
                    print('Done Move')


                    #cmd_vel.publish(traj[i])


        while not rospy.is_shutdown():
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
        

        
        

