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

import inter_proba_1 as ip
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

                labels_ws = ip.get_lables(array_2d)
                props = ip.regionprops(labels_ws)
                i=0

                for p in props:
                    
                    (conturs,flannen_contours) = ip.get_counturs_from_label(p.coords,array_2d.shape)

                    pth = ip.PathFinder(conturs,array_2d,2,3)
                    covered_points = pth.get_route(True,False)
                    points = []
                    
                    for cp in covered_points:
                        points.append([cp[0]*self.map.info.resolution,cp[1]*self.map.info.resolution,math.pi/2])

                    corrected_conturs = []
                    for fc in flannen_contours:
                        corrected_conturs.append([fc[0]*self.map.info.resolution,fc[1]*self.map.info.resolution,math.pi/2])
                    
                    way_point = WayPoint(corrected_conturs, points, str(i))                    
                    self.way_points.append(way_point)

                    # for c in z:
                    #     canvas = cv2.polylines(array_2d_rgb, [c], True, (255, 0, 0) , 1)

                    # cv2.putText(img,str(i), (c[0][0][0],c[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,255,255),2)
                    # cv2.imshow("drawCntsImg.jpg", array_2d)
                    # cv2.waitKey(0)
                    i+=1
                    
                    # break
            if len(self.way_points)>0:
                self.way_points[1].display()
            r.sleep()
        #rospy.spin()

    def callback(self, data):
        self.map = data
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        


if __name__ == '__main__':
    c = Path_Creator()

