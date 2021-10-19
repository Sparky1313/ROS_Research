#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('wiggle_dance')

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(1)
move = Twist()
move.linear.x = -0.10


def wiggle():
    pub.publish(move)


while not rospy.is_shutdown():
    wiggle()
    rate.sleep()

