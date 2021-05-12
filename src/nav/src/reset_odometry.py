#!/usr/bin/env python3


import rospy
from odom_publisher.srv import reset_odom_srv

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