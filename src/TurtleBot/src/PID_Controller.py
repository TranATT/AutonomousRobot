#!/usr/bin/env python
#Problem 1

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

Kp_gain = 1
Ki_gain = 0.1
Kd_gain = 0

position_x = 0
position_y = 0
rotation = 0

move_cmd = Twist()
cmd_vel = None

goal_x = 0
goal_y = 0
goal_angle = 0

mode = None


def moveModeOne(): #Mode 1
    global goal_x
    global goal_y
    global goal_angle
    global Kp_gain
    global move_cmd
    global position_x
    global position_y
    global rotation

    error_distance = sqrt(pow(goal_x - position_x, 2) + pow(goal_y - position_y, 2))
    distnation_angle = np.arctan((goal_y - position_y)/(goal_x - position_x))

    error_angle = np.arctan2(np.sin(distnation_angle - rotation), np.cos(distnation_angle - rotation))
   
    error_a = goal_angle - rotation


    move_cmd.linear.x = min(Kp_gain * error_distance, 0.1)
    move_cmd.angular.z = min(Kp_gain * error_angle, 0.1) 
    cmd_vel.publish(move_cmd)        

def applyController():
    global goal_x
    global goal_y
    global rotation
    global position_x
    global position_y
    global goal_angle

    error_linear = sqrt(pow(goal_x - position_x, 2) + pow(goal_y - position_y, 2))
    error_angular = goal_angle - rotation
    #if (error_linear > 0.05) and (abs(error_angular) > pi / 180):
    moveModeOne()

def angularController(destination_angle): #Angular for Mode 0
    global move_cmd
    global rotation
    global Kp_gain
    error_angle = np.arctan2(np.sin(destination_angle - yaw), np.cos(destination_angle - yaw))
    move_cmd.angular.z = min(Kp_gain * error_angle, 0.1)
    move_cmd.linear.x = 0
    cmd_vel.publish(move_cmd)

def linearController(): #Linear for Mode 0
    global move_cmd
    global Kp_gain
    global goal_x
    global goal_y
    global position_x
    global position_y

    error_distance = sqrt(pow(goal_x - position_x, 2) + pow(goal_y - position_y, 2))
    move_cmd.linear.x = min(Kp_gain * error_distance, 0.1)
    move_cmd.angular.z = 0
    cmd_vel.publish(move_cmd)


def applyController0(): #Mode 0
    global goal_x
    global goal_y
    global goal_angle
    global Kp_gain
    global position_x
    global position_y
    global rotation
    global yaw
    destination_angle = np.arctan((goal_y - position_y)/(goal_x - position_x))
    error_linear = sqrt(pow(goal_x - position_x, 2) + pow(goal_y - position_y, 2))
    error_angular = destination_angle - yaw
    error_angular_goal = goal_angle - yaw
    angular_flag = 0
    linear_flag = 0
    goal_flag = 0
    if (abs(error_angular) > pi / 180 and angular_flag == 0): #Step 1 - Turn until error is within bounds
        if (goal_x < position_x):
            destination_angle = destination_angle + pi
        angularController(destination_angle)
    else:
        angular_flag = 1

    if (abs(error_linear) > pi / 180 and linear_flag == 0 and angular_flag == 1): #Step 2 - Move until error is within bounds
        linearController()
    else:
        linear_flag = 1

    if (abs(error_angular_goal) > pi / 180 and goal_flag == 0 and angular_flag == 1 and linear_flag == 1):
        angularController(goal_angle)
    else:
        goal_flag = 1

    if (goal_flag == 1):
        goal_flag = 0
        angular_flag = 0
        linear_flag = 0

def pose_update(msg):
    global position_x
    global position_y
    global rotation
    global goal_x
    global goal_y
    global goal_angle
    global mode
    global yaw
    #print(msg.pose[1])


    position_x = msg.pose[1].position.x
    position_y = msg.pose[1].position.y
    rotation = msg.pose[1].orientation.z
    temp1 = msg.pose[1].orientation
    orientation_list = [temp1.x, temp1.y, temp1.z, temp1.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    error = sqrt(pow(goal_x - position_x, 2) + pow(goal_y - position_y, 2))
    #print(error)
    if (error > 0.05):
        if (mode == 0):
            applyController0()
        if (mode == 1):
            applyController()
    if (goal_x != 0) and (error < 0.05):
        print("Target achieved")
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        cmd_vel.publish(move_cmd) 
    
def setReferencePoint(data):
    global goal_x
    global goal_y
    global goal_angle
    global mode

    goal_x = data.data[0]
    goal_y = data.data[1]
    goal_angle = data.data[2]
    mode = data.data[3]

if __name__ == '__main__':
  

    rospy.init_node('PID_Control_US', anonymous=False)
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    rospy.Subscriber('/gazebo/model_states', ModelStates, pose_update)
    rospy.Subscriber('/reference_pose', Float64MultiArray, setReferencePoint)

    r = rospy.Rate(10)    
    
   
    #print("Enter final x position")
    #x_final = input()
    #print("Enter final y position")
    #y_final = input()
    #print("Enter final angle position")
    #angle_final = input()
    #final = [x_final, y_final, angle_final]
    #final_position = np.array(final)

    #goal_x = final_position[0]
    #goal_y = final_position[1]
    #goal_angle = final_position[2]

    try:
        print("Running Controller...Please wait")
		#time.sleep(1)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

    except:
        rospy.loginfo("Finished Target.")
         
