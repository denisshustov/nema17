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

import numpy as np
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

def get_conturs():
    try:
        rospy.loginfo("try call service /contur_creator/get_conturs")

        get_conturs = rospy.ServiceProxy('/contur_creator/get_conturs', conturs_srv)
        rqt = conturs_srvRequest('')
        resp = get_conturs(rqt)
        
        rospy.loginfo("call service /contur_creator/get_conturs success")
        return resp
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)

def get_way_points():
    processed = []
    way_points = []

    conturs = get_conturs()#helper
    my_coordinated = position_by_transorm(5)#path_helper
    if my_coordinated == None:
        rospy.loginfo("Contur position not found by transform map=>base_link")
        return

    current_countur = get_conturs_by_xy(my_coordinated[0], my_coordinated[1])# path_helper
    if current_countur == None or \
        current_countur.conturs == None or \
        len(current_countur.conturs) == 0 or \
        len(current_countur.conturs[0].points) == 0:

        rospy.loginfo("Contur not by position x={},y={}".format(my_coordinated[0],my_coordinated[1]))
        return

    contur_id = current_countur.conturs[0].contur_id
    while True:
        path = get_path(contur_id)#path_helper

        #contur_to_path[contur_id] = path

        if path == None or len(path.points)==0:
            rospy.loginfo("Points is empty for contur {}".format(contur_id))
            return

        way_points.append(WayPoint(current_countur.conturs[0].points, path.points, contur_id))
        if contur_id not in processed:            
            processed.append(contur_id)
        
        connected_conturs = get_by_next_by_id(contur_id)
        if connected_conturs == None or len(connected_conturs.conturs)==0:
            rospy.loginfo("Points is empty for contur {}".format(contur_id))
            break

        not_proc_connected_conturs = [c for c in connected_conturs.conturs if not c.contur_id in processed]
        if len(not_proc_connected_conturs)==0:
            c_ids = np.array([cc.contur_id for cc in conturs.conturs])
            diffs = list(set(c_ids) - set(processed))
            if len(diffs)==0:
                break

            contur_id = diffs[0]                
        else:
            contur_id = not_proc_connected_conturs[0].contur_id
        
    return way_points