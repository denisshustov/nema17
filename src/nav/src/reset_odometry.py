#!/usr/bin/env python3


import rospy
from math import sin, cos, pi
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import tf
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from odom_publisher.srv import reset_odom_srv, reset_odom_srvResponse

def reset_odom_client():
    rospy.wait_for_service('reset_odom_service')
    try:
        reset_odom = rospy.ServiceProxy('reset_odom_service', reset_odom_srv)
        resp = reset_odom(True)
        return resp.response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
     print("Reset odom response %s"%(reset_odom_client()))