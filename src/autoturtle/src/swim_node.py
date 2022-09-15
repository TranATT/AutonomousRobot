#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import random

def pose_callback(inp):
    global p
    p = inp
    p.x = round(p.x, 3)
    p.y - round(p.y, 3)

def figure8():
    global p
    global r
    global initial_X
    global initial_Y
    global move_message
    global pos_message

    m = Twist()
    linear_velocity = random.randint(-2, 2)
    m.linear.x = linear_velocity
    angular_velocity = random.randint(-2, 2)
    m.angular.z = angular_velocity

    complete = 0
    while(True):
        if (((p.x > initial_X + 0.2) or (p.x > initial_Y - 0.2)) and ((p.y < initial_Y - 0.2) or (p.y > initial_X + 0.2))):
                complete = 1 #checks if complete circle has been made
        elif( ((p.x < initial_X + 0.2) and (p.x > initial_Y - 0.2)) and ((p.y > initial_Y - 0.2) and (p.y < initial_Y + 0.2)) ):
            #once first circle is complete create the second one
                if(complete == 1):
                    complete = 2
                    m.angular.z = m.angular.z * -1
        
        move_message.publish(m)
        r.sleep()
    rospy.spin()


if __name__ == '__main__':
    global p
    global r
    global initial_X
    global initial_Y
    global move_message
    global pos_message
    try:
        rospy.init_node('swim_node')
        p = Pose()
        r = rospy.Rate(1)
        initial_X = 5.544445
        initial_Y = 5.544445
        move_message = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        pos_message = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
        figure8()
    except rospy.ROSInterruptException:
        pass

