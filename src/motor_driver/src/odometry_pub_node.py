#!/usr/bin/env python


import time
import sys
from time import sleep
import rospy
#from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf

#get poset from
#rostopic echo /slam_out_pose
#geometry_msgs/PoseStamped
class OdomPub:
    def TwistCallback(self,wheel_odom):#pose, 
        self.Twist = wheel_odom 
        odom = Odometry()
        odom.header.stamp = rospy.get_rostime()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        for i in range(0,36):
            #odom.pose.covariance[i] = pose.pose.covariance[i]
            odom.twist.covariance[i] = 0        

        # odom.pose.pose.position.x = pose.pose.position.x
        # odom.pose.pose.position.y = pose.pose.position.y
        # odom.pose.pose.position.z = pose.pose.position.z

        # odom.pose.pose.orientation.x = pose.pose.orientation.x
        # odom.pose.pose.orientation.y = pose.pose.orientation.y
        # odom.pose.pose.orientation.z = pose.pose.orientation.z
        # odom.pose.pose.orientation.w = pose.pose.orientation.w

        odom.twist.twist.linear.x = wheel_odom.linear.x
        odom.twist.twist.linear.y = wheel_odom.linear.y
        odom.twist.twist.linear.z = wheel_odom.linear.z

        odom.twist.twist.angular.x = wheel_odom.angular.x
        odom.twist.twist.angular.y = wheel_odom.angular.y
        odom.twist.twist.angular.z = wheel_odom.angular.z
        self.pub_odom.publish(odom)

    def __init__(self):
        self.Twist = None
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.init_node("odometry_pub_node")
        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.pose_sub = rospy.Subscriber("/ppp/cmd_vel", Twist, self.TwistCallback)
    
    def run(self):

        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():

            if self.Twist is not None:
                odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)

                self.odom_broadcaster.sendTransform(
                    (self.Twist.linear.x, self.Twist.linear.y, self.Twist.linear.z),
                    odom_quat,
                    rospy.Time.now(),
                    "base_link",
                    "odom"
                )
            r.sleep()

def main():
    odomPub = OdomPub()
    odomPub.run()


if __name__ == '__main__':
    main()










#wheel_odom_sub = Subscriber("/slam_out_pose", PoseStamped)
#ats = ApproximateTimeSynchronizer([pose_sub], queue_size=5, slop=0.1)#wheel_odom_sub
#ats.registerCallback(callback)



#rospy.spin()
