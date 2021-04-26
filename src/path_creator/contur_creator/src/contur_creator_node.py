#!/usr/bin/env python3

import roslib;
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Quaternion, Point

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry

import cv2
import os
import sys
import math
import random
import numpy as np

sys.path.append(os.path.join(sys.path[0], '../../libraries'))
from Conturs import *

# from PathFinder import *

# from WayPoint import *

from path_creator.srv import way_points_srv, way_points_srvResponse
from path_creator.srv import conturs_srvResponse, conturs_srv
from map_contur_msg.msg import map_contur_msg

class Contur_Creator:
    def __init__(self):
        rospy.init_node("contur_creator")
        rospy.Subscriber("/map", OccupancyGrid, self.callback)
        
        self.robot_diametr = 0.3
        self.robot_center_pixel_x = 10
        self.robot_center_pixel_y = 10        
        self.map = None        
        self.array_map = []
        self.conutrs = []
        self.find_conutrs_in_progress = False
        self.find_path_in_progress = None

        self.srv = rospy.Service('contur_creator/get_conturs', conturs_srv, self.get_conturs)
        self.srv1 = rospy.Service('contur_creator/get_by_id', conturs_srv, self.get_by_id)
        rospy.loginfo("contur_creator Starting...")
        self.rate = rospy.get_param('~rate',100.0)
        rospy.spin()

    def check_errors(self):
        if self.map == None:
            return way_points_srvResponse(error_code="MAP_NOT_READY")
        if self.find_conutrs_in_progress:
            return way_points_srvResponse(error_code="FIND_CONTURS_IN_PROGRESS")
        # if self.conutrs  == None:
        #     return way_points_srvResponse(error_code="CONTURS_NOT_READY")
        # if self.way_points == None:
        #     return way_points_srvResponse(error_code="WAY_POINTS_NOT_READY")
        return None

    def get_by_id(self, request):
        error = self.check_errors()
        if error != None:
            return error
        if len(request.contur_id)==0:
            return way_points_srvResponse(error_code="contur_id_IS_EMPTY")
        if self.find_path_in_progress == request.contur_id:
            return way_points_srvResponse(error_code="FIND_PATH_FOR_CURRENT_ID_IN_PROGRESS")

        if len(self.conutrs) == 0:
            self._get_conturs()
        if len(self.conutrs) == 0:
            return way_points_srvResponse(error_code="CONTURS_NOT_FOUND")
        
        contur = None
        for c in self.conutrs:
            if c.id == request.contur_id:
               contur = c
               break
        if contur == None:
            return way_points_srvResponse(error_code="contur_id_NOT_FOUND")
           
        return conturs_srvResponse(conturs = [contur])

    def get_conturs(self, request):
        error = self.check_errors()
        if error != None:
            return error
        if len(self.conutrs) == 0:
            self._get_conturs()
        if len(self.conutrs) == 0:
            return way_points_srvResponse(error_code="CONTURS_NOT_FOUND")
        
        all_conturs = []
        for cc in self.conutrs:
            cor_con = []
            for c in cc.corrected_contur:
                cor_con.append(Point(c[0]*self.map.info.resolution,c[1]*self.map.info.resolution,0))
            all_conturs.append(map_contur_msg(contur_id = cc.id, points = cor_con))
        return conturs_srvResponse(conturs = all_conturs)

    def _get_conturs(self):
        if self.map != None and self.conutrs == [] and not self.find_conutrs_in_progress:
            self.find_conutrs_in_progress = True
            self.array_map = self.map_to_array()
            # self.save_array_to_file(self.array_map)

            cnt_inst = Conturs(self.array_map)
            self.conutrs = cnt_inst.get_conturs(skip_area_less_than = 40)
            inter = cnt_inst.get_intersections(5)
            self.find_conutrs_in_progress = False
            return self.conutrs
        return []

    
    def save_array_to_file(self, arr):
        np.savetxt('/home/den/catkin_ws/src/path_creator/test/test1.txt', arr, fmt='%d')
    def load_array_to_file(self):
        return np.loadtxt('/home/den/catkin_ws/src/path_creator/test/test1.txt', dtype=int)

    def map_to_array(self):
        result = np.reshape(self.map.data, (-1, self.map.info.width))

        np.place(result, result ==255, -100500)
        np.place(result, result ==0, 255)
        np.place(result, result ==-1, 0)
        np.place(result, result ==100, 0)
        np.place(result, result ==-100500, 0)
        result = np.uint8(result)
        #array_2d_rgb = backtorgb = cv2.cvtColor(result,cv2.COLOR_GRAY2RGB)
        return result

    def callback(self, data):
        self.map = data
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

if __name__ == '__main__':
    c = Contur_Creator()
    # c.go()
