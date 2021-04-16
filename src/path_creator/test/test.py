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

def exisits_not_processed(conturs, intersects, ignore_without_relations = True):
    result = True
    for i in intersects:
        if not i.is_processed:
            return True
    return False


def get_next_contur2(current, go_from = None):
    if len(current.children)>0:
        for c in current.children:
            if not c.is_processed:
                return [c]

        #all children processed
        res = None
        for c in current.children:
            if go_from != None:
                if go_from.id != c.id:
                    res = get_next_contur2(c, current)
            else:
                res = get_next_contur2(c, current)
            if res != None:
                res.append(c)
                res.append(current)
                return res

    return None
#/home/pi/catkin_ws/src/path_creator/test/img/map.jpg
img = cv2.imread(os.path.join(sys.path[0], 'img')+'/mymap_22.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
flag, image = cv2.threshold(gray, 205, 255, cv2.THRESH_BINARY)

cnt_inst = Conturs(image)
cnts = cnt_inst.get_conturs()
inter = cnt_inst.get_intersections()

cnt_inst.show(img)

i=1
start_point = None
current_contur = cnts[0]

while True:
    if not current_contur.is_processed:
        pth = PathFinder(current_contur.contur, image, 8, 2, start_point=start_point, debug_mode=True)
        covered_points = pth.get_route()
        if len(covered_points)>0:
            start_point = covered_points[len(covered_points)-1]
            current_contur.is_processed = True
        else:
            print('covered_points is empty, id={}!!!'.format(current_contur.id))

        pth.show_mounting_point(img)
        pth.show_path_point(img)

    current_conturs = get_next_contur2(current_contur)
    if current_conturs == None:
        break
    current_contur = current_conturs[0]

    i+=1

# pth.show_grid(image)
# pth.show_mounting_point(image)
# pth.show_path_point(image)
cv2.imshow("drawCntsImg2.jpg", img)
cv2.waitKey(0)

