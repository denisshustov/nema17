#!/usr/bin/env python


import time
import sys
from time import sleep
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist, Odometry


def callback(self, vel_msg):
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = self.frame_id
    odom.twist.twist.linear.x = vel_msg.twist.linear.x

    odom.twist.twist.linear.y = vel_msg.twist.linear.y
    odom.twist.twist.linear.z = vel_msg.twist.linear.z

    odom.twist.twist.angular.x = vel_msg.twist.angular.x
    odom.twist.twist.angular.y = vel_msg.twist.angular.y
    odom.twist.twist.angular.z = vel_msg.twist.angular.z


rospy.init_node("odometry_pub_node")
ros_sub_twist = rospy.Subscriber("/ppp/cmd_vel", Twist, callback)
pub = rospy.Publisher("/odom", Odometry, queue_size=10)

rospy.sleep(3)
