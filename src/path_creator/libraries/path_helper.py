#!/usr/bin/env python3

import rospy
import actionlib
import geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import tf.transformations
import tf

from actionlib_msgs.msg import GoalStatus

from std_msgs.msg import ByteMultiArray

from path_creator.srv import way_points_srv, way_points_srvRequest
from path_creator.srv import conturs_by_point_srv, conturs_by_point_srvRequest
import threading

import os
import sys
# sys.path.append(os.path.join(sys.path[0], '../../libraries'))
from marker_lib import *

from WayPoint import *

from path_creator.srv import way_points_srv, conturs_srv, way_points_srvRequest, conturs_srvRequest
from path_creator.srv import way_points_srv, way_points_srvRequest
from path_creator.srv import conturs_by_point_srv, conturs_by_point_srvRequest

def position_by_transorm(wait_seconds = 100):
    tf_listener = tf.TransformListener()
    trans = None
    i=0

    while True:
        try:
            (trans,rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except Exception as ex:
            rospy.loginfo("Transform map => base_link NOT FOUND!!! ERROR:{}".format(ex))
        
        if trans != None or i>wait_seconds:
            break
        rospy.sleep(1)
        i+=1

    return trans

def get_conturs_by_xy(x,y):
        try:
            rospy.loginfo("try call service /contur_creator/get_by_xy")

            get_by_xy = rospy.ServiceProxy('/contur_creator/get_by_xy', conturs_by_point_srv)
            rqt = conturs_by_point_srvRequest(Point(x,y,0))
            resp = get_by_xy(rqt)
            
            rospy.loginfo("call service /contur_creator/get_by_xy success")
            return resp
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

def get_path(contur_id, current_position_x = None, current_position_y = None):
    try:
        rospy.loginfo("try call service /path_creator/get_by_id {}".format(contur_id))

        get_path = rospy.ServiceProxy('/path_creator/get_by_id', way_points_srv)
        rqt = way_points_srvRequest(contur_id, None, None)
        resp = get_path(rqt)
        rospy.loginfo("call service /path_creator/get_by_id {} success".format(contur_id))

        return resp
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)


def get_by_next_by_id(contur_id):
    try:
        rospy.loginfo("try call service /contur_creator/get_by_next_by_id {}".format(contur_id))

        get_next_countur = rospy.ServiceProxy('/contur_creator/get_by_next_by_id', conturs_srv)
        rqt = conturs_srvRequest(contur_id)
        resp = get_next_countur(rqt)
        rospy.loginfo("call service /contur_creator/get_by_next_by_id {} success".format(contur_id))

        return resp
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)