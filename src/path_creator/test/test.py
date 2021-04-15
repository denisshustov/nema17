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

sys.path.append('/home/pi/catkin_ws/src/path_creator/script')
from inter_proba_1 import *


#/home/pi/catkin_ws/src/path_creator/test/img/map.jpg
img = cv2.imread('/home/pi/catkin_ws/src/path_creator/test/img/mymap_22.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
flag, image = cv2.threshold(gray, 205, 255, cv2.THRESH_BINARY)
cnts = get_conturs(image)
i=0

for cnt in cnts:
    pth = PathFinder(cnt,image,7,2,visualize=True,visualize_grid=False)
    qqq = pth.get_route()
    for c in cnt:
        if cv2.contourArea(c) > 100:
            cv2.drawContours(image, [c], -1, (0, 0, 255), 1, 1)
    i+=1

cv2.imshow("drawCntsImg2.jpg", image)
cv2.waitKey(0)

