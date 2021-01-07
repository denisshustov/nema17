#!/usr/bin/env python


import rospy
from math import sin, cos, pi
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import tf
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from motor_driver.srv import reset_odom_srv, reset_odom_srvResponse

class Cmd_to_odom:

    def __init__(self):
        rospy.init_node("Cmd_to_odom")
        self.nodename = rospy.get_name()
        self.srv = rospy.Service('reset_odom_service', reset_odom_srv, self.reset_odom)

        rospy.loginfo("Cmd_to_odom Starting...")
        
        self.rate = rospy.get_param('~rate',1000.0)
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_footprint_id = rospy.get_param('~base_footprint_id', 'base_footprint')
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.init_variables()

        rospy.Subscriber("/ppp/real_cmd_vel", Twist, self.callback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size = 500)        
        rospy.loginfo("Cmd_to_odom success")

    def init_variables(self):
        self.x = 0.0
        self.y = 0.0
        self.theta  = 0.0
        self.delta_x = 0
        self.delta_y = 0
        self.dt = 0

        self.cmdVel = None
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

    def reset_odom(self, request):
        if request.reset:
            rospy.loginfo("reset all odom variables")
            self.init_variables()

            self.send(0.0, 0.0, 0.0, 0.0)

            return reset_odom_srvResponse("Ok")
        else:
            rospy.loginfo("NOT reset odom")
            return reset_odom_srvResponse("Nothing")

    def run(self):
        r = rospy.Rate(self.rate)
        zero_processed = False

        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            self.dt = (self.current_time - self.last_time).to_sec()
            self.last_time = self.current_time

            if self.cmdVel is None:
                continue
            
            linear_velocity_x = self.cmdVel.linear.x
            linear_velocity_y = self.cmdVel.linear.y
            angular_velocity_z = self.cmdVel.angular.z

            is_all_zero = self.cmdVel.linear.x == 0 and \
                self.cmdVel.linear.y == 0 and \
                self.cmdVel.angular.z == 0

            if zero_processed and is_all_zero:
                continue

            zero_processed = is_all_zero

            # rospy.loginfo("x={},y={},z={}".format( \
            #     linear_velocity_x,linear_velocity_y,angular_velocity_z))
            
            self.x += (linear_velocity_x * cos(self.theta) - linear_velocity_y * sin(self.theta)) * self.dt
            self.y += (linear_velocity_x * sin(self.theta) + linear_velocity_y * cos(self.theta)) * self.dt
            self.theta += angular_velocity_z * self.dt
            self.send(linear_velocity_x, linear_velocity_y, angular_velocity_z, self.theta)

            r.sleep()
       
    def send(self, lv_x, lv_y, lv_z, theta):
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

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

        odom.twist.twist.linear.x = lv_x
        odom.twist.twist.linear.y = lv_y
        odom.twist.twist.linear.z = 0
        
        odom.twist.twist.angular.y = 0
        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.z = lv_z

        self.odomPub.publish(odom)


    def callback(self, cmdVel):
        self.cmdVel = cmdVel
        

if __name__ == '__main__':
    cmd_to_odom = Cmd_to_odom()
    cmd_to_odom.send(0.0, 0.0, 0.0, 0.0)
    cmd_to_odom.run()