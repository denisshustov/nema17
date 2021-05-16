#!/usr/bin/env python3

import rospy
import geometry_msgs
import tf.transformations
from visualization_msgs.msg import Marker

class Marker_lib():

    def get_marker(self, index, point, is_text=False):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = index+1000
        marker.ns = "hz_namespace"
        if not is_text:
            marker.type = marker.CYLINDER 
        else:
            marker.type = marker.TEXT_VIEW_FACING

        marker.action = marker.ADD
        marker.text = str(index)
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 10.0
        marker.color.b = 0.0
        # marker.points = [Point(0,0, 0.), Point(0,0 , 0.9),]

        q = tf.transformations.quaternion_from_euler(0, 0, point[2], axes='sxyz')
        marker.pose.orientation = geometry_msgs.msg.Quaternion(*q)
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0
        return marker

    def get_marker2(self, index, points):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = index+1000
        marker.ns = "hz_namespace"
        marker.type = marker.LINE_STRIP

        marker.action = marker.ADD
        marker.text = str(index)
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 10.0
        marker.color.b = 0.0
        marker.points = points

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        return marker

    def init_markers(self):
        if len(self.points)>1:
            self.marker = self.get_marker2(1,self.points)
        else:
            self.marker = self.get_marker(1,self.points[0], True)
        self.marker_pub.publish(self.marker)


    def publish(self):
        self.marker_pub.publish(self.marker)

    def __init__(self, topic_name, points):
        self.marker_pub = rospy.Publisher(topic_name, Marker, queue_size = 1)

        self.points = points
        self.init_markers()
