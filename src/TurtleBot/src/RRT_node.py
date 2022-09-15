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

import numpy as np
from matplotlib import cm
from scipy.misc import imread
import random, sys, math, os.path
from nav_msgs.msg import OccupancyGrid

from rrt import find_path_RRT
import cv2

current_map = OccupancyGrid()
cmd_vel = None

x_start_real = 0
y_start_real = 0
x_goal_real = 0
y_goal_real = 0

resolution = 0
x_origin = 0
y_origin = 0

x_start_index = 0
y_start_index = 0
x_goal_index = 0
y_goal_index = 0

def convert1Dto2D(data):
    global current_map
    global resolution
    global x_origin
    global y_origin
    resolution = data.info.resolution
    x_origin = data.info.origin.position.x
    y_origin = data.info.origin.position.y
    current_map = np.reshape(data.data, (data.info.width, data.info.height))
    print(current_map)

def get_index_from_coordinates(x, y):
    global x_origin
    global y_origin
    global resolution
    x_output = int(round((x-x_origin)/resolution))
    y_output = int(round((y-y_origin)/resolution))
    #x_output = int(round((x_origin-x)/resolution))
    #y_output = int(round((y_origin-y)/resolution))

    return x_output, y_output

def get_coordinates_from_index(x_index, y_index):
    global x_origin
    global y_origin
    global resolution
    x_real = x_origin + (x_index + 0.5) * resolution
    y_real = y_origin + (y_index + 0.5) * resolution

    return x_real, y_real

def startGoal(data):
    global x_start_real
    global y_start_real
    global x_goal_real
    global y_goal_real
    global x_start_index
    global y_start_index
    global x_goal_index
    global y_goal_index
    global current_map

    x_start_real = data.data[0]
    y_start_real = data.data[1]
    x_goal_real = data.data[2]
    y_goal_real = data.data[3]
    
    start = Float64MultiArray()
    goal = Float64MultiArray()

    x_start_index, y_start_index = get_index_from_coordinates(x_start_real, y_start_real)
    x_goal_index, y_goal_index = get_index_from_coordinates(x_goal_real, y_goal_real)
    
    
    start, goal = ([x_start_index, y_start_index], [x_goal_index, y_goal_index])
    
   # start = [78, 140]
   # goal = [128, 217]
    #tempSx, tempSy = get_coordinates_from_index(78, 140)
    #tempGx, tempGy = get_coordinates_from_index(128, 217)
    #print(tempSx)
    #print(tempSy)
    #print(tempGx)
    #print(tempGy)
    print(start)
    print(goal)
    
    print('HI') 
    path, graph = find_path_RRT(start, goal, current_map)
    print(path)
    arr = []
    trajectory = Float64MultiArray()

    for i in range(len(path)):
        path_x, path_y = get_coordinates_from_index(path[i][0], path[i][1])
        arr.append(path_x)
        arr.append(path_y)
    
    trajectory.data = arr

    
    print(trajectory.data)

    cmd_vel.publish(trajectory)
    print('Done')

if __name__ == '__main__':
    try:
        rospy.init_node('RRT_node', anonymous=False)
        rospy.Subscriber('/map', OccupancyGrid, convert1Dto2D)
        rospy.Subscriber('/start_goal', Float64MultiArray, startGoal)
        cmd_vel = rospy.Publisher('/trajectory', Float64MultiArray, queue_size = 10)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass


