#!/usr/bin/env python


import rospy
from math import sin, cos, pi
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import tf
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

if __name__ == '__main__':

    rospy.init_node("reset_odometry")
    self.nodename = rospy.get_name()
    rospy.loginfo("Cmd_to_odom Starting...")

    self.odomPub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom = Odometry()
    odom.header.stamp = self.current_time
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'

    odom.pose.pose.position.x = self.x
    odom.pose.pose.position.y = self.y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = Quaternion(*odom_quat)

    # odom.twist.twist.linear.x = 0
    # odom.twist.twist.linear.y = 0
    odom.twist.twist.linear.x = linear_velocity_x
    odom.twist.twist.linear.y = linear_velocity_y
    odom.twist.twist.linear.z = 0

    odom.twist.twist.angular.y = 0
    odom.twist.twist.angular.x = 0
    odom.twist.twist.angular.z = angular_velocity_z_

    self.odomPub.publish(odom)