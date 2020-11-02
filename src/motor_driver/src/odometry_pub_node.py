#!/usr/bin/env python


import time
import sys
from time import sleep
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber

#get poset from
#rostopic echo /slam_out_pose
#geometry_msgs/PoseStamped

def callback(self, pose, wheel_odom):
    odom = Odometry()
    odom.header.stamp = wheel_odom.header.stamp
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_id"

    for i in range(0,36):
        odom.pose.covariance[i] = pose.pose.covariance[i]
        odom.twist.covariance[i] = 0
    

    odom.pose.pose.position.x = pose.pose.position.x
    odom.pose.pose.position.y = pose.pose.position.y
    odom.pose.pose.position.z = pose.pose.position.z

    odom.pose.pose.orientation.x = pose.pose.orientation.x
    odom.pose.pose.orientation.y = pose.pose.orientation.y
    odom.pose.pose.orientation.z = pose.pose.orientation.z
    odom.pose.pose.orientation.w = pose.pose.orientation.w

    odom.twist.twist.linear.x = wheel_odom.twist.linear.x
    odom.twist.twist.linear.y = wheel_odom.twist.linear.y
    odom.twist.twist.linear.z = wheel_odom.twist.linear.z

    odom.twist.twist.angular.x = wheel_odom.twist.angular.x
    odom.twist.twist.angular.y = wheel_odom.twist.angular.y
    odom.twist.twist.angular.z = wheel_odom.twist.angular.z
    pub_odom.publish(odom)

rospy.init_node("odometry_pub_node")

pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)

pose_sub = Subscriber("/ppp/cmd_vel", Twist)
wheel_odom_sub = Subscriber("/slam_out_pose", PoseStamped)
ats = ApproximateTimeSynchronizer([pose_sub, wheel_odom_sub], queue_size=5, slop=0.1)
ats.registerCallback(callback)



rospy.spin()
