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

from PathFinder import *

# from WayPoint import *

from path_creator.srv import way_points_srv, way_points_srvResponse, conturs_srvResponse, conturs_srv, conturs_srvRequest
from path_creator.srv import conturs_by_point_srv, conturs_by_point_srvResponse, conturs_by_point_srvRequest

from map_contur_msg.msg import map_contur_msg

class Path_Creator:
    def __init__(self):
        rospy.init_node("path_creator")
        rospy.Subscriber("/map", OccupancyGrid, self.callback)
        self.map = None
        self.array_map = []

        self.cnt_inst = None
        self.robot_diametr = 0.3
        self.robot_center_pixel_x = 10
        self.robot_center_pixel_y = 10        
        self.way_points = []
        self.covered_points = []
        self.conutrs = []
        self.find_conutrs_in_progress = False
        self.find_path_in_progress = None

        self.srv = rospy.Service('path_creator/get_by_id', way_points_srv, self.get_path_by_id)
        self.srv1 = rospy.Service('contur_creator/get_conturs', conturs_srv, self.get_conturs)
        self.srv2 = rospy.Service('contur_creator/get_by_id', conturs_srv, self.get_contur_by_id)
        self.srv2 = rospy.Service('contur_creator/get_by_xy', conturs_by_point_srv, self.get_contur_by_xy)

        rospy.loginfo("path_creator Starting...")
        self.rate = rospy.get_param('~rate',100.0)
        rospy.spin()

    def callback(self, data):
        self.map = data

#--------------path-----------
    def get_path_by_id(self, request):
        if len(request.contur_id)==0:
            return way_points_srvResponse(error_code="contur_id_IS_EMPTY")
        if self.find_path_in_progress == request.contur_id:
            return way_points_srvResponse(error_code="FIND_PATH_FOR_CURRENT_ID_IN_PROGRESS")

        if len(self.conutrs) == 0:
            self._get_conturs()
        if len(self.conutrs) == 0:
            return conturs_srvResponse(error_code="CONTURS_NOT_FOUND")
        
        contur = None
        for c in self.conutrs:
            if c.id == request.contur_id:
               contur = c
               break
        if contur == None:
            return way_points_srvResponse(error_code="contur_id_NOT_FOUND")

        result = self._get_path(contur)            
        return way_points_srvResponse(points = result, contur_id = request.contur_id)
    
    def _get_path(self, contur):
        self.find_path_in_progress = contur.id
        pth = PathFinder(contur.contur, self.array_map.shape, 5, 1, start_point=None, debug_mode=False)
        self.covered_points = pth.get_route()
        points = []
        for cp in self.covered_points:
            points.append(Point(cp[0]*self.map.info.resolution,cp[1]*self.map.info.resolution,0))
        self.find_path_in_progress = None
        return points
#--------------path-----------
    
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

#-------------contur------------------

    def get_contur_by_xy(self, request):
        if self.map == None:
            return conturs_by_point_srvResponse(error_code="MAP_NOT_FOUND")

        if request.point == None:
            return conturs_by_point_srvResponse(error_code="FIND_CONTURS_IN_PROGRESS")
        
        if len(self.conutrs) == 0:
            self._get_conturs()
        if len(self.conutrs) == 0:
            return conturs_by_point_srvResponse(error_code="CONTURS_NOT_FOUND")
        
        contur_by_point = self.cnt_inst.get_contur_by_coord(request.point.x,request.point.y)
        if contur_by_point == None:
            return conturs_by_point_srvResponse(error_code="CONTURS_NOT_FOUND_BY_POSITION")

        cor_con = self.contur_to_point(contur_by_point)
        return conturs_by_point_srvResponse(contur_id = contur_by_point.id, conturs = [map_contur_msg(points = cor_con)])

    def contur_to_point(self, contur):
        result = []
        for c in contur.corrected_contur:
            result.append(Point(c[0]*self.map.info.resolution,c[1]*self.map.info.resolution,0))
        return result

    def get_contur_by_id(self, request):
        if self.find_conutrs_in_progress:
            return way_points_srvResponse(error_code="FIND_CONTURS_IN_PROGRESS")
        if len(request.contur_id)==0:
            return conturs_srvResponse(error_code="contur_id_IS_EMPTY")
        if self.find_path_in_progress == request.contur_id:
            return conturs_srvResponse(error_code="FIND_PATH_FOR_CURRENT_ID_IN_PROGRESS")

        if len(self.conutrs) == 0:
            self._get_conturs()
        if len(self.conutrs) == 0:
            return conturs_srvResponse(error_code="CONTURS_NOT_FOUND")
        
        cor_con = []
        for contur in self.conutrs:
            if contur.id == request.contur_id:
                cor_con = self.contur_to_point(contur)
                # for c in contur.corrected_contur:
                #     cor_con.append(Point(c[0]*self.map.info.resolution,c[1]*self.map.info.resolution,0))
                break
        if len(cor_con)==0:
            return conturs_srvResponse(error_code="contur_id_NOT_FOUND")
           
        return conturs_srvResponse(conturs = [map_contur_msg(contur_id = request.contur_id, points = cor_con)])

    def get_conturs(self, request):
        if self.find_conutrs_in_progress:
            return way_points_srvResponse(error_code="FIND_CONTURS_IN_PROGRESS")
        if len(self.conutrs) == 0:
            self._get_conturs()
        if len(self.conutrs) == 0:
            return conturs_srvResponse(error_code="CONTURS_NOT_FOUND")
        
        all_conturs = []
        for cc in self.conutrs:
            cor_con = self.contur_to_point(cc)
            # for c in cc.corrected_contur:
            #     cor_con.append(Point(c[0]*self.map.info.resolution,c[1]*self.map.info.resolution,0))
            all_conturs.append(map_contur_msg(contur_id = cc.id, points = cor_con))
        return conturs_srvResponse(conturs = all_conturs)

    def _get_conturs(self):
        if self.map != None and self.conutrs == [] and not self.find_conutrs_in_progress:
            self.find_conutrs_in_progress = True
            self.array_map = self.map_to_array()
            # self.save_array_to_file(self.array_map)

            self.cnt_inst = Conturs(self.array_map)
            self.conutrs = self.cnt_inst.get_conturs(skip_area_less_than = 40)
            inter = self.cnt_inst.get_intersections(5)
            self.find_conutrs_in_progress = False
            return self.conutrs
        return []

#-------------contur------------------

if __name__ == '__main__':
    c = Path_Creator()
    # c.go()


# def go(self):
#         r = rospy.Rate(100)        
#         while not rospy.is_shutdown():
#             if self.map != None and self.way_points == []:
#                 #z = self.map.info.resolution #meters / pixel 
#                 robot_in_pixels = self.robot_diametr / self.map.info.resolution #6x6

#                 array_map = self.map_to_array()
#                 self.save_array_to_file(array_map)

#                 cnt_inst = Conturs(array_map)
#                 self.conutrs = cnt_inst.get_conturs(skip_area_less_than = 40)

#                 inter = cnt_inst.get_intersections(5)
#                 current_contur = self.conutrs[0]

#                 i=1
#                 start_point = None

#                 while True:
#                     if not current_contur.is_processed:
#                         pth = PathFinder(current_contur.contur, array_map, 5, 1, start_point=start_point, debug_mode=False)
#                         self.covered_points = pth.get_route()
#                         if len(self.covered_points)>0:
#                             start_point = self.covered_points[len(self.covered_points)-1]
#                             current_contur.is_processed = True
#                         else:
#                             print('covered_points is empty, id={}!!!'.format(current_contur.id))

#                         #--- multiply to self.map.info.resolution, for correcting ---

#                         points = []
#                         for cp in self.covered_points:
#                             points.append([cp[0]*self.map.info.resolution,cp[1]*self.map.info.resolution,math.pi/2])

#                         i=0
#                         cor_con = []
#                         for cc in current_contur.corrected_contur:
#                             cor_con.append((cc[0]*self.map.info.resolution,cc[1]*self.map.info.resolution))
#                             i+=1

#                         #--- multiply to self.map.info.resolution, for correcting ---

#                         way_point = WayPoint(cor_con, points, str(i))                    
#                         self.way_points.append(way_point)

#                     current_conturs = cnt_inst.get_contur_in_order(current_contur,None,[],[],0)
#                     if current_conturs == None:
#                         break
#                     if len(current_conturs)==0:
#                         break
#                     current_contur = current_conturs[0]

#                     i+=1
#             if len(self.way_points)>0:
#                 for w in self.way_points:
#                     w.display()
#             r.sleep()
#         #rospy.spin()
    