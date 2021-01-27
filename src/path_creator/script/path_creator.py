#!/usr/bin/env python

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

class Path_Creator:
    def __init__(self):
        rospy.init_node("Cmd_to_odom")
        rospy.Subscriber("/map", OccupancyGrid, self.callback)
        
        self.robot_diametr = 0.3
        self.robot_center_pixel_x = 10
        self.robot_center_pixel_y = 10
        
        self.map = None
        
        r = rospy.Rate(100)        
        while not rospy.is_shutdown():
            if self.map != None:
                z = self.map.info.resolution #meters / pixel 
                robot_in_pixels = self.robot_diametr / self.map.info.resolution #6x6

                w = self.map.info.width
                h = self.map.info.height
                d = self.map.data
                array_2d = np.reshape(self.map.data, (-1, self.map.info.width))

            r.sleep()
        #rospy.spin()

    def callback(self, data):
        self.map = data
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        


if __name__ == '__main__':
    c = Path_Creator()

