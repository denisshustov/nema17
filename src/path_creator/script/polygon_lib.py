#!/usr/bin/env python3

import roslib 
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PolygonStamped, Point32
from std_msgs.msg import Header

class Ploygon_lib():

    def __init__(self):
        self.marker_pub = rospy.Publisher('visualization_polygon', PolygonStamped, queue_size = 111)
        self.points = []

    def SquarePolygon(self,header):        
        p = PolygonStamped()
        p.header = header
        p.polygon.points = self.points
        return p

    def add_points(self, points):
        for p in points:
            self.points.append(Point32(x=p[0], y=p[1]))

    def publish(self):
        header = Header()
        header.frame_id = "world"
        header.stamp = rospy.Time.now()

        p = self.SquarePolygon(header)
        self.marker_pub.publish(p)