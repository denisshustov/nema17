#!/usr/bin/env python3

import roslib;
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import random
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Header

import cv2
import os
import sys

from inter_proba_1 import *
from WayPoint import *

class Path_Creator:
    def __init__(self):
        rospy.init_node("path_creator")
        rospy.Subscriber("/map", OccupancyGrid, self.callback)
        
        self.robot_diametr = 0.3
        self.robot_center_pixel_x = 10
        self.robot_center_pixel_y = 10
        
        self.map = None
        
        self.way_points = []

        r = rospy.Rate(100)        
        while not rospy.is_shutdown():
            if self.map != None and self.way_points == []:
                #z = self.map.info.resolution #meters / pixel 
                robot_in_pixels = self.robot_diametr / self.map.info.resolution #6x6

                array_map = self.map_to_array()

                conturs = get_conturs(array_map,self.map.info.resolution)
                # zzz = cv2.convexHull([conturs[0][0],conturs[1][0]], False)
                i=0
                start_point = None
                for (contour, corrected_contur) in conturs:
                    pth = PathFinder(contour,array_map,3,1,neibor_distance=8,start_point=start_point)
                    covered_points = pth.get_route()
                    start_point = covered_points[len(covered_points)-1]

                    points = []
                    for cp in covered_points:
                        points.append([cp[0]*self.map.info.resolution,cp[1]*self.map.info.resolution,math.pi/2])
                
                    way_point = WayPoint(corrected_contur, points, str(i))                    
                    self.way_points.append(way_point)
                    i+=1

            if len(self.way_points)>0:
                for w in self.way_points:
                    w.display()
            r.sleep()
        #rospy.spin()
        
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

