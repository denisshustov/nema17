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

sys.path.append(os.path.join(sys.path[0], '../script'))
from inter_proba_1 import *

sys.path.append(os.path.join(sys.path[0], '../script/libraries'))
from Conturs import *

#/home/pi/catkin_ws/src/path_creator/test/img/map.jpg
img = cv2.imread('/home/pi/catkin_ws/src/path_creator/test/img/mymap_22.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
flag, image = cv2.threshold(gray, 205, 255, cv2.THRESH_BINARY)

cnt_inst = Conturs(image)
cnts = cnt_inst.get_conturs()
cnt_inst.show(img)
i=0

start_point = None
for (cnt, corrected_contur) in cnts:
    pth = PathFinder(cnt, image, 8, 2, start_point=start_point, debug_mode=True)
    covered_points = pth.get_route()
    start_point = covered_points[len(covered_points)-1]
    pth.show_mounting_point(img)
    pth.show_path_point(img)

    i+=1
# pth.show_grid(image)
# pth.show_mounting_point(image)
# pth.show_path_point(image)
cv2.imshow("drawCntsImg2.jpg", img)
cv2.waitKey(0)

