#!/usr/bin/env python3

import roslib 
import rospy
import actionlib
import geometry_msgs
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import tf.transformations
import time
from math import radians, pi
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from actionlib_msgs.msg import GoalStatus

from std_msgs.msg import ByteMultiArray

class Marker_lib():

    def get_marker(self, index, is_text=False):
        marker = Marker()
        marker.header.frame_id = "world"
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

        q = tf.transformations.quaternion_from_euler(0, 0, self.goal_list[index][2], axes='sxyz')
        marker.pose.orientation = geometry_msgs.msg.Quaternion(*q)
        marker.pose.position.x = self.goal_list[index][0]
        marker.pose.position.y = self.goal_list[index][1]
        marker.pose.position.z = 0
        return marker

    def init_markers(self):
        index = 0
        for g in self.goal_list:
            self.markerArray.markers.append(self.get_marker(index))
            # self.markerArray.markers.append(self.get_marker(index, True))
            index += 1
        
        print('Publish markers')
        self.marker_pub.publish(self.markerArray)

    def publish_markers(self):
        self.marker_pub.publish(self.markerArray)

    def add_points(self, goal_list):
        self.goal_list = goal_list
        self.current_goal_index = 0
        self.init_markers()

    def __init__(self):
        self.marker_pub = rospy.Publisher('visualization_marker', MarkerArray, queue_size = 1)
        self.markerArray = MarkerArray()
