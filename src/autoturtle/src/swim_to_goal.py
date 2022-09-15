#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt
from turtlesim.msg import Pose

class SwimTurtle:
    def __init__(self):
        rospy.init_node('swim_to_goal', anonymous=True)
        self.r = rospy.Rate(10)
        self.message_velocity = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.message_pose = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.current = Pose()

    #Euclidean distance
    def error_position(self, goal_message):
        return sqrt(pow((goal_message.x - self.current.x), 2) + pow((goal_message.y - self.current.y), 2))

    def error_angle(self, goal_message):
        return atan2(goal_message.y - self.current.y, goal_message.x - self.current.x)

    def linear_velocity(self, goal_message, K_x=1.5):
        return K_x * self.error_position(goal_message)
    
    def angular_velocity(self, goal_message, K_z=4):
        return (self.error_angle(goal_message) - self.current.theta) * K_z

    def pose_callback(self, inp):
        self.current = inp
        self.current.x = round(self.current.x, 3)
        self.current.y = round(self.current.y, 3)


    def swimToGoal(self):
        goal_message = Pose()
        velocity = Twist()
        
        goal_message.x = float(input("Input x goal: "))
        goal_message.y = float(input("Input y goal: "))
        
        while(self.error_position(goal_message) >= 0.5):
            velocity.linear.x = self.linear_velocity(goal_message)
            velocity.angular.z = self.angular_velocity(goal_message)

            self.message_velocity.publish(velocity)

            self.r.sleep()

        #robot done moving
        velocity.linear.x = 0
        velocity.linear.z = 0
        self.message_velocity.publish(velocity)


        



if __name__ == '__main__':
    try:
        t = SwimTurtle()
        while(True): 
            t.swimToGoal()
    except rospy.ROSInterruptException:
        pass


