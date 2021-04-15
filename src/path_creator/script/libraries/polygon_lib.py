#!/usr/bin/env python3

import roslib 
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PolygonStamped, Point32
from std_msgs.msg import Header

class Ploygon_lib():

    def __init__(self, topic_name, points):
        self.points=[]  
        self.marker_pub = rospy.Publisher(topic_name, PolygonStamped, queue_size = 111)
        for p in points:
            self.points.append(Point32(x=p[0], y=p[1]))

    def SquarePolygon(self,header, points):        
        p = PolygonStamped()
        p.header = header
        p.polygon.points = points
        return p

    def publish(self):
        header = Header()
        header.frame_id = "world"
        header.stamp = rospy.Time.now()

        polygon = self.SquarePolygon(header,self.points)
        self.marker_pub.publish(polygon)