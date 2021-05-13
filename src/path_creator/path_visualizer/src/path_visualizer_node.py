#!/usr/bin/env python3

import rospy

import os
import sys
import numpy as np

sys.path.append(os.path.join(sys.path[0], '../../libraries'))
# from Conturs import *

# from PathFinder import *

from WayPoint import *

from path_creator.srv import way_points_srv, conturs_srv, way_points_srvRequest, conturs_srvRequest

class Path_Visualizer:
    def __init__(self):
        rospy.init_node("path_visualizer_node")
        
        rospy.wait_for_service('contur_creator/get_conturs')
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

    def get_path(self, contur_id):
        try:
            rospy.loginfo("try call service /path_creator/get_by_id {}".format(contur_id))

            get_path = rospy.ServiceProxy('/path_creator/get_by_id', way_points_srv)
            rqt = way_points_srvRequest(contur_id)
            resp = get_path(rqt)
            rospy.loginfo("call service /path_creator/get_by_id {} success".format(contur_id))

            return resp
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

    def process(self):
        self.conturs = self.get_conturs()
        i=0
        for c in self.conturs.conturs:
            p = v.get_path(c.contur_id)
            self.contur_to_path[c.contur_id] = p
            way_point = WayPoint(c.points, p.points, str(i))                    
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
