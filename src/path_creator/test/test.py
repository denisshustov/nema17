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

sys.path.append(os.path.join(sys.path[0], '../script/libraries'))

from PathFinder import *
from Conturs import *

def get_next_contur(current, conturs, intersects):
    for i in intersects:
        if i.id == current.id and i.children!=None and len(i.children)>0:
            not_processed_child=[ch for ch in i.children if not ch.is_processed]
            if not_processed_child!=None and len(not_processed_child)>0:
                return not_processed_child[0]
            else:
                processed_child=[ch for ch in i.children if ch.is_processed]
                return processed_child[0]

    return None

#/home/pi/catkin_ws/src/path_creator/test/img/map.jpg
img = cv2.imread(os.path.join(sys.path[0], 'img')+'/mymap_22.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
flag, image = cv2.threshold(gray, 205, 255, cv2.THRESH_BINARY)

cnt_inst = Conturs(image)
cnts = cnt_inst.get_conturs()
inter = cnt_inst.get_intersections()

cnt_inst.show(img)

#------------------------------------------------

i=0

start_point = None
for cnt in cnts:
    pth = PathFinder(cnt.contur, image, 8, 2, start_point=start_point, debug_mode=True)
    covered_points = pth.get_route()
    start_point = covered_points[len(covered_points)-1]
    pth.show_mounting_point(img)
    pth.show_path_point(img)

    i+=1
#------------------------------------------------
# i=1
# start_point = None
# current_contur = cnts[0]

# while True:
#     if not current_contur.is_processed:
#         pth = PathFinder(current_contur.contur, image, 8, 2, start_point=start_point, debug_mode=True)
#         covered_points = pth.get_route()
#         if len(covered_points)>0:
#             start_point = covered_points[len(covered_points)-1]
#             current_contur.is_processed = True
#         else:
#             print('covered_points is empty, id={}!!!'.format(current_contur.id))

#         pth.show_mounting_point(img)
#         pth.show_path_point(img)

#     current_contur = get_next_contur(current_contur, cnts, inter)
#     if current_contur==None or i==17:
#         break
#     i+=1

# pth.show_grid(image)
# pth.show_mounting_point(image)
# pth.show_path_point(image)
cv2.imshow("drawCntsImg2.jpg", img)
cv2.waitKey(0)

