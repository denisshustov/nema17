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

sys.path.append(os.path.join(sys.path[0], '../libraries'))

from PathFinder import *
from Conturs import *

def load_array_to_file():
    return np.loadtxt(sys.path[0]+'/test1.txt', dtype=np.uint8)


#/home/pi/catkin_ws/src/path_creator/test/img/map.jpg
img = cv2.imread(os.path.join(sys.path[0], 'img')+'/mymap_22.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
flag, image = cv2.threshold(gray, 205, 255, cv2.THRESH_BINARY)

#image = img = load_array_to_file()
cnt_inst = Conturs(image)
cnts = cnt_inst.get_conturs(40)

# xxx = cnt_inst.merge('0','2')
# xxx = cnt_inst.merge('0','5')
# xxx = cnt_inst.merge('4','6')
# xxx = cnt_inst.merge('4','0')

# xxx = cnt_inst.merge('1','3')
# xxx = cnt_inst.merge('1','9')
# xxx = cnt_inst.merge('1','10')
# xxx = cnt_inst.merge('1','12')
# xxx = cnt_inst.merge('1','13')

# xxx = cnt_inst.merge('11','14')
# xxx = cnt_inst.merge('11','8')

inter = cnt_inst.get_intersections(2)
x=756
y=299
current_contur = cnts[0] #cnt_inst.get_contur_by_coord(x, y)

# img = cv2.normalize(img, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
#np.uint8(img)
image = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)

cnt_inst.show(image)

i=1
start_point = {0:500,1:150}
cv2.circle(image, (start_point[0], start_point[1]),10, (0,0,255), -1)

while True:
    if not current_contur.is_processed:
        pth = PathFinder(current_contur.contur, image.shape, 5, 1, start_point=start_point, debug_mode=True)
        covered_points = pth.get_route()
        if len(covered_points)>0:
            start_point = covered_points[len(covered_points)-1]
            current_contur.is_processed = True
        else:
            print('covered_points is empty, id={}!!!'.format(current_contur.id))

        # pth.show_mounting_point(img)
        pth.show_path_point(image)

    current_conturs = cnt_inst.get_contur_in_order(current_contur,None,[],[],0)

    if current_conturs == None or i>len(cnts):
        break
    if len(current_conturs)==0:
        break
    current_contur = current_conturs[0]

    i+=1

# pth.show_grid(image)
# pth.show_mounting_point(image)
pth.show_path_point(image)
cv2.imshow("drawCntsImg2.jpg", image)
cv2.waitKey(0)

