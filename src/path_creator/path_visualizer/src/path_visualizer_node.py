#!/usr/bin/env python3

import rospy

import os
import sys
import numpy as np
import tf.transformations
import tf
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

sys.path.append(os.path.join(sys.path[0], '../../libraries'))
# from Conturs import *

# from PathFinder import *

from WayPoint import *

from path_creator.srv import way_points_srv, conturs_srv, way_points_srvRequest, conturs_srvRequest
from path_creator.srv import way_points_srv, way_points_srvRequest
from path_creator.srv import conturs_by_point_srv, conturs_by_point_srvRequest

class Path_Visualizer:
    def __init__(self):
        rospy.init_node("path_visualizer_node")
        
        rospy.wait_for_service('contur_creator/get_conturs')
        rospy.wait_for_service('contur_creator/get_by_xy')
        rospy.wait_for_service('path_creator/get_by_id')

        rospy.loginfo("path_visualizer_node Starting...")
        self.rate = rospy.get_param('~rate',10.0)
                
        self.conturs = []
        self.contur_to_path = {}
        self.way_points = []
    
    def get_conturs(self):
        try:
            rospy.loginfo("try call service /contur_creator/get_conturs")

            get_conturs = rospy.ServiceProxy('/contur_creator/get_conturs', conturs_srv)
            rqt = conturs_srvRequest('')
            resp = get_conturs(rqt)
            
            rospy.loginfo("call service /contur_creator/get_conturs success")
            return resp
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

    def get_path(self, contur_id, current_position_x = None, current_position_y = None):
        try:
            rospy.loginfo("try call service /path_creator/get_by_id {}".format(contur_id))

            get_path = rospy.ServiceProxy('/path_creator/get_by_id', way_points_srv)
            rqt = way_points_srvRequest(contur_id, None, None)
            resp = get_path(rqt, current_position_x, current_position_y)
            rospy.loginfo("call service /path_creator/get_by_id {} success".format(contur_id))

            return resp
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

    def get_conturs_by_xy(self,x,y):
        try:
            rospy.loginfo("try call service /contur_creator/get_by_xy")

            get_by_xy = rospy.ServiceProxy('/contur_creator/get_by_xy', conturs_by_point_srv)
            rqt = conturs_by_point_srvRequest(Point(x,y,0))
            resp = get_by_xy(rqt)
            
            rospy.loginfo("call service /contur_creator/get_by_xy success")
            return resp
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

    #TODO TEST THIS!!!!
    def process2(self):
        i=0
        tf_listener = tf.TransformListener()
        trans = None

        while True:
            try:
                (trans,rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except Exception as ex:
                rospy.loginfo("Transform map => base_link NOT FOUND!!! ERROR:{}".format(ex))
            
            if trans != None or i>100:
                break
            rospy.sleep(1)
            i+=1

        c = self.get_conturs_by_xy(trans[0], trans[1])
        p = v.get_path(c.contur_id)
        self.contur_to_path[c.contur_id] = p

        current_conturs = []
        current_points = []

        if c == None or len(c.points)==0:
            rospy.loginfo("Conturs is empty for contur {}".format(c.contur_id))
        else:
            current_conturs = c.points
        if p == None or len(p.points)==0:
            rospy.loginfo("Points is empty for contur {}".format(c.contur_id))
        else:
            current_points = p.points

        way_point = WayPoint(current_conturs, current_points, c.contur_id)
        self.way_points.append(way_point)
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            for w in self.way_points:
                w.display()

            r.sleep()

    def process(self):
        self.conturs = self.get_conturs()
        i=0
        for c in self.conturs.conturs:
            p = v.get_path(c.contur_id)
            self.contur_to_path[c.contur_id] = p

            current_conturs = []
            current_points = []

            if c == None or len(c.points)==0:
                rospy.loginfo("Conturs is empty for contur {}".format(c.contur_id))
            else:
                current_conturs = c.points
            if p == None or len(p.points)==0:
                rospy.loginfo("Points is empty for contur {}".format(c.contur_id))
            else:
                current_points = p.points

            way_point = WayPoint(current_conturs, current_points, c.contur_id)
            self.way_points.append(way_point)
            i+=1

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            for w in self.way_points:
                w.display()

            r.sleep()
#-------------contur------------------

if __name__ == '__main__':
    v = Path_Visualizer()
    v.process()
