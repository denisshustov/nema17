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
from path_helper import *

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

    #TODO TEST THIS!!!!
    def process2(self):
        processed = {}

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

            self.contur_to_path[contur_id] = path

            if path == None or len(path.points)==0:
                rospy.loginfo("Points is empty for contur {}".format(contur_id))
                return

            way_point = WayPoint(current_countur.conturs[0].points, path.points, contur_id)
            self.way_points.append(way_point)
            if contur_id not in processed:
                processed[contur_id]=1
            else:
                processed[contur_id]+=1

            connected_conturs = get_by_next_by_id(contur_id)
            if connected_conturs == None or len(connected_conturs.conturs)==0:
                rospy.loginfo("Points is empty for contur {}".format(contur_id))
                break #!!!!!!!!!!!!!!!!!!!

            not_proc_connected_conturs = [c for c in connected_conturs.conturs if not c.contur_id in processed]
            if len(not_proc_connected_conturs)==0:
                #GET not visined conturs!!!!!!!!!!!!!!!!!!!!
                #GET not visined conturs!!!!!!!!!!!!!!!!!!!!
                #GET not visined conturs!!!!!!!!!!!!!!!!!!!!
                #GET not visined conturs!!!!!!!!!!!!!!!!!!!!
                #GET not visined conturs!!!!!!!!!!!!!!!!!!!!
                #GET not visined conturs!!!!!!!!!!!!!!!!!!!!
                #GET not visined conturs!!!!!!!!!!!!!!!!!!!!
                #GET not visined conturs!!!!!!!!!!!!!!!!!!!!
                #GET not visined conturs!!!!!!!!!!!!!!!!!!!!
                #GET not visined conturs!!!!!!!!!!!!!!!!!!!!
                #GET not visined conturs!!!!!!!!!!!!!!!!!!!!
                rospy.loginfo("!!!!Points is empty for contur {}".format(contur_id))
                break

            contur_id = not_proc_connected_conturs[0].contur_id

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
    v.process2()
