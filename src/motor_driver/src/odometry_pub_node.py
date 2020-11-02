#!/usr/bin/env python


import time
import sys
from time import sleep
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist, Odometry, PoseWithCovarianceStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber

#get poset from
#rostopic echo /slam_out_pose

def callback(self, pose, wheel_odom):
    odom = Odometry()
    odom.header.stamp = wheel_odom.header.stamp
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_footprint"

    for i in range(0,36):
        odom.pose.covariance[i] = pose.pose.covariance[i]
        odom.twist.covariance[i] = 0
    

    odom.pose.pose.position.x = pose.pose.pose.position.x
    odom.pose.pose.position.y = pose.pose.pose.position.y
    odom.pose.pose.position.z = pose.pose.pose.position.z

    odom.pose.pose.orientation.x = pose.pose.pose.orientation.x
    odom.pose.pose.orientation.y = pose.pose.pose.orientation.y
    odom.pose.pose.orientation.z = pose.pose.pose.orientation.z
    odom.pose.pose.orientation.w = pose.pose.pose.orientation.w

    odom.twist.twist.linear.x = wheel_odom.twist.linear.x
    odom.twist.twist.linear.y = wheel_odom.twist.linear.y
    odom.twist.twist.linear.z = wheel_odom.twist.linear.z

    odom.twist.twist.angular.x = wheel_odom.twist.angular.x
    odom.twist.twist.angular.y = wheel_odom.twist.angular.y
    odom.twist.twist.angular.z = wheel_odom.twist.angular.z
    pub = rospy.Publisher("/odom", Odometry, queue_size=10)

rospy.init_node("odometry_pub_node")

pose_sub = Subscriber("/ppp/cmd_vel", Twist)
wheel_odom_sub = Subscriber("/poseWithCovarianceStamped", Twist)
ats = ApproximateTimeSynchronizer([pose_sub, wheel_odom_sub], queue_size=5, slop=0.1))
ats.registerCallback(gotimage)



rospy.sleep(3)
