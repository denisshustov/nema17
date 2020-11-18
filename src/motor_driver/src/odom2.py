#!/usr/bin/env python


import rospy
from math import sin, cos, pi
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import tf
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#from std_msgs.msg import Int16, Int32, Int64, UInt32
#https://gist.github.com/grassjelly/b06aaf5e73019868236eeb425ca60f76
#https://answers.ros.org/question/241602/get-odometry-from-wheels-encoders/
#https://answers.ros.org/question/296112/odometry-message-for-ackerman-car/
#https://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
#https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf

class DiffTf:

    def __init__(self):
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # basefootprint /the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
               
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        
        self.x = 0.0
        self.y = 0.0
        self.theta  = 0.0
        self.delta_theta = 0

        self.vx = 0.1
        self.vy = -0.1
        self.vth = 0.1

        self.quaternion = Quaternion()
        self.cmdVel = None
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        rospy.Subscriber("/ppp/cmd_vel", Twist, self.callback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        
    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()            
            self.delta_theta = self.cmdVel.linear.z

            dt = (self.current_time - self.last_time).to_sec()
            delta_x = (self.cmdVel.linear.x * cos(self.theta) - self.cmdVel.linear.y * sin(self.theta)) * dt
            delta_y = (self.cmdVel.linear.x * sin(self.theta) + self.cmdVel.linear.y * cos(self.theta)) * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += self.delta_theta

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                self.current_time,
                "base_link",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(odom_quat))
            # set the velocity
            
            odom.twist.twist = Twist(Vector3(self.cmdVel.linear.x, self.cmdVel.linear.y, 0), Vector3(0, 0, self.vth))

            # publish the message
            self.odomPub.publish(odom)

            self.last_time = self.current_time
            r.sleep()
       
    def callback(self, cmdVel):
        self.cmdVel = cmdVel

if __name__ == '__main__':
    """ main """
    diffTf = DiffTf()
    diffTf.run()