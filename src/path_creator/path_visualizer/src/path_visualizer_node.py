#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

from nav_msgs.msg import OccupancyGrid

import os
import sys
import numpy as np

sys.path.append(os.path.join(sys.path[0], '../../libraries'))
from Conturs import *

from PathFinder import *

# from WayPoint import *

from path_creator.srv import way_points_srv, way_points_srvResponse, conturs_srvResponse, conturs_srv, way_points_srvRequest
from path_creator.srv import conturs_by_point_srv, conturs_by_point_srvResponse

from map_contur_msg.msg import map_contur_msg

class Path_Visualizer:
    def __init__(self):
        rospy.init_node("path_visualizer_node")
        
        rospy.wait_for_service('contur_creator/get_conturs')
        rospy.wait_for_service('path_creator/get_by_id')

        rospy.loginfo("path_visualizer_node Starting...")
        self.rate = rospy.get_param('~rate',10.0)
                
        self.conturs = []

        # rospy.spin()
    
    def get_conturs(self):
        try:
            get_conturs = rospy.ServiceProxy('/contur_creator/get_conturs', way_points_srv)
            rqt = way_points_srvRequest('')
            resp = get_conturs(rqt)
            return resp.response
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)


#-------------contur------------------

if __name__ == '__main__':
    v = Path_Visualizer()
    conturs = v.get_conturs()
    q=123
    z=123
