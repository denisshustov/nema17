#!/usr/bin/env python


import rospy
from math import sin, cos, pi
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import tf
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


#from std_msgs.msg import Int16, Int32, Int64, UInt32
#https://gist.github.com/grassjelly/b06aaf5e73019868236eeb425ca60f76
#https://answers.ros.org/question/241602/get-odometry-from-wheels-encoders/
#https://answers.ros.org/question/296112/odometry-message-for-ackerman-car/
#https://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
#https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf

class Cmd_to_odom:

    def __init__(self):
        super().__init__('reset_odom_service')
        self.srv = self.create_service(Bool, 'reset_odom_service', self.reset_odom)

        rospy.init_node("Cmd_to_odom")
        self.nodename = rospy.get_name()
        rospy.loginfo("Cmd_to_odom Starting...")
        
        self.rate = rospy.get_param('~rate',10.0)
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_footprint_id = rospy.get_param('~base_footprint_id', 'base_footprint')

        self.init_variables()

        rospy.Subscriber("/ppp/cmd_vel", Twist, self.callback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        rospy.loginfo("Cmd_to_odom success")

    def init_variables(self):
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.x = 0.0
        self.y = 0.0
        self.theta  = 0.0
        self.delta_theta = 0

        self.cmdVel = None
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()


    def self.reset_odom(self, request):
        rospy.loginfo("reset all odom variables")
        self.init_variables()

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.cmdVel is None: continue

            linear_velocity_x = self.cmdVel.linear.x
            linear_velocity_y = self.cmdVel.linear.y
            angular_velocity_z_ = self.cmdVel.angular.z

            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()
            self.last_time = self.current_time

            self.delta_theta = angular_velocity_z_ * dt
            
            # 0.073 radius
            delta_x = (linear_velocity_x * cos(self.theta) - linear_velocity_y * sin(self.theta)) * dt
            delta_y = (linear_velocity_x * sin(self.theta) + linear_velocity_y * cos(self.theta)) * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += self.delta_theta
            
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                self.current_time,
                self.base_footprint_id,
                self.odom_frame_id
            )

            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = self.odom_frame_id
            odom.child_frame_id = self.base_footprint_id

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = Quaternion(*odom_quat)

            odom.twist.twist.linear.x = linear_velocity_x
            odom.twist.twist.linear.y = linear_velocity_y
            odom.twist.twist.linear.z = 0
            
            odom.twist.twist.angular.y = 0
            odom.twist.twist.angular.x = 0
            odom.twist.twist.angular.z = angular_velocity_z_

            self.odomPub.publish(odom)
            
            r.sleep()
       
    def callback(self, cmdVel):
        self.cmdVel = cmdVel

if __name__ == '__main__':
    cmd_to_odom = Cmd_to_odom()
    cmd_to_odom.run()