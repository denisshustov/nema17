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

sys.path.append(os.path.join(sys.path[0], 'libraries'))
from Conturs import *

from PathFinder import *

from WayPoint import *

from path_creator.srv import * #way_points_srv, way_points_srvResponse


class Path_Creator:
    def __init__(self):
        rospy.init_node("path_creator")
        rospy.Subscriber("/map", OccupancyGrid, self.callback)
        
        self.robot_diametr = 0.3
        self.robot_center_pixel_x = 10
        self.robot_center_pixel_y = 10        
        self.map = None        
        self.way_points = []
        self.covered_points = []

        self.conutrs = []
        self.find_conutrs_in_progress = False

        self.srv = rospy.Service('path_creator/get_by_id', way_points_srv, self.get_by_id)
        self.srv2 = rospy.Service('path_creator/get_all', way_points_srv, self.get_all)
        rospy.loginfo("path_creator Starting...")
        self.rate = rospy.get_param('~rate',100.0)
        rospy.spin()

    def check_errors(self):
        if self.map == None:
            return way_points_srvResponse(points=[],error_code="MAP_NOT_READY",error_description="")
        if self.find_conutrs_in_progress:
            return way_points_srvResponse(points=[],error_code="FIND_CONTURS_IN_PROGRESS",error_description="")
        # if self.conutrs  == None:
        #     return way_points_srvResponse(points=[],error_code="CONTURS_NOT_READY",error_description="")
        # if self.way_points == None:
        #     return way_points_srvResponse(points=[],error_code="WAY_POINTS_NOT_READY",error_description="")
        return None

    def get_all(self, request):
        error = self.check_errors()
        if error != None:
            return error
        
        self.get_conturs()
        result = []
        result.append(Point(1,2,0))
        result.append(Point(3,4,0))
        return way_points_srvResponse(result)

    def get_by_id(self, request):
        error = self.check_errors()
        if error != None:
            return error

        self.get_conturs()

        if request.contur_id:
            result = []
            result.append(Point(1,2,0))
            result.append(Point(3,4,0))
            
            return way_points_srvResponse(points=result)
        return way_points_srvResponse(points=[])
    
    def get_conturs(self):
        if self.map != None and self.conutrs == [] and not self.find_conutrs_in_progress:
            self.find_conutrs_in_progress = True
            array_map = self.map_to_array()
            self.save_array_to_file(array_map)

            cnt_inst = Conturs(array_map)
            self.conutrs = cnt_inst.get_conturs(skip_area_less_than = 40)
            inter = cnt_inst.get_intersections(5)
            self.find_conutrs_in_progress = False
            return self.conutrs
        return []

    def go(self):
        r = rospy.Rate(100)        
        while not rospy.is_shutdown():
            if self.map != None and self.way_points == []:
                #z = self.map.info.resolution #meters / pixel 
                robot_in_pixels = self.robot_diametr / self.map.info.resolution #6x6

                array_map = self.map_to_array()
                self.save_array_to_file(array_map)

                cnt_inst = Conturs(array_map)
                self.conutrs = cnt_inst.get_conturs(skip_area_less_than = 40)

                inter = cnt_inst.get_intersections(5)
                current_contur = self.conutrs[0]

                i=1
                start_point = None

                while True:
                    if not current_contur.is_processed:
                        pth = PathFinder(current_contur.contur, array_map, 5, 1, start_point=start_point, debug_mode=False)
                        self.covered_points = pth.get_route()
                        if len(self.covered_points)>0:
                            start_point = self.covered_points[len(self.covered_points)-1]
                            current_contur.is_processed = True
                        else:
                            print('covered_points is empty, id={}!!!'.format(current_contur.id))

                        #--- multiply to self.map.info.resolution, for correcting ---

                        points = []
                        for cp in self.covered_points:
                            points.append([cp[0]*self.map.info.resolution,cp[1]*self.map.info.resolution,math.pi/2])

                        i=0
                        cor_con = []
                        for cc in current_contur.corrected_contur:
                            cor_con.append((cc[0]*self.map.info.resolution,cc[1]*self.map.info.resolution))
                            i+=1

                        #--- multiply to self.map.info.resolution, for correcting ---

                        way_point = WayPoint(cor_con, points, str(i))                    
                        self.way_points.append(way_point)

                    current_conturs = cnt_inst.get_contur_in_order(current_contur,None,[],[],0)
                    if current_conturs == None:
                        break
                    if len(current_conturs)==0:
                        break
                    current_contur = current_conturs[0]

                    i+=1
            if len(self.way_points)>0:
                for w in self.way_points:
                    w.display()
            r.sleep()
        #rospy.spin()
    
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
    c = Path_Creator()
    # c.go()