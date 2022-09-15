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

goal = Float64MultiArray()

if __name__ == '__main__':
    try:
        rospy.init_node('test', anonymous=False)
        p = rospy.Publisher('/target_pose', Float64MultiArray, queue_size=10)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            goal_x = float(input("Enter x goal:"))
            goal_y = float(input("Enter y goal:"))
        
            goal_send = [goal_x, goal_y]
            goal.data = goal_send
            p.publish(goal)
            print('Sent')

        
    except rospy.ROSInterruptException:
        pass

    

