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

                w = self.map.info.width
                h = self.map.info.height
                d = self.map.data
                array_2d = np.reshape(self.map.data, (-1, self.map.info.width))

                np.place(array_2d, array_2d ==255, -100500)
                np.place(array_2d, array_2d ==0, 255)
                np.place(array_2d, array_2d ==-1, 0)
                np.place(array_2d, array_2d ==100, 0)
                np.place(array_2d, array_2d ==-100500, 0)
                array_2d = np.uint8(array_2d)
                array_2d_rgb = backtorgb = cv2.cvtColor(array_2d,cv2.COLOR_GRAY2RGB)

                conturs = get_conturs(array_2d)
                i=0

                for cnt in conturs:
                    pth = PathFinder(cnt,array_2d,2,1)
                    covered_points = pth.get_route()

                    points = []
                    for cp in covered_points:
                        points.append([cp[0]*self.map.info.resolution,cp[1]*self.map.info.resolution,math.pi/2])
                    
                    flannen_contours=[]
                    for idx, val in enumerate(cnt):
                        for idx1, val1 in enumerate(val):
                            for idx2, val2 in enumerate(val1):
                                flannen_contours.append((cnt[idx][idx1][idx2][0],cnt[idx][idx1][idx2][1]))

                    corrected_conturs = []
                    for fc in flannen_contours:
                        corrected_conturs.append([fc[0]*self.map.info.resolution,fc[1]*self.map.info.resolution,math.pi/2])

                    way_point = WayPoint(corrected_conturs, points, str(i))                    
                    self.way_points.append(way_point)
                    i+=1

            if len(self.way_points)>0:
                for w in self.way_points:
                    w.display()
            r.sleep()
        #rospy.spin()

    def callback(self, data):
        self.map = data
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        


if __name__ == '__main__':
    c = Path_Creator()

