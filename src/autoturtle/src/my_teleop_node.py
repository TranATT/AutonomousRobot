#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
import click

def move(linear_velocity, angular_velocity):
    move_message = Twist()
    r = rospy.Rate(10)
    move_message.linear.x = linear_velocity
    move_message.angular.z = angular_velocity
    pubV = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size =10)
    pubV.publish(move_message)
    r.sleep()
    


if __name__ == '__main__':
    try:
        rospy.init_node('my_teleop_node')
        print('Move the turtle with WASD or press q to quit')
        while(True):
            c = click.getchar()
            if c == 'w':
                move(3, 0)
            elif c == 'a':
                move(0, 1)
            elif c == 's':
                move(-3, 0)
            elif c == 'd':
                move(0, -1)
            elif c == 'q':
                print('Quit')
                break
            else:
                print('Invalid Input')
    except rospy.ROSInterruptException:
        pass




