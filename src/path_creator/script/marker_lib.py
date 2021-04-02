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

    def get_marker(self, index):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = index
        marker.ns = "hz_namespace"
        marker.type = marker.TEXT_VIEW_FACING #SPHERE
        marker.action = marker.ADD
        marker.text = str(index)
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

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

# if __name__ == '__main__':
#     try:
#         positions = [
#                 [8.77073478699, 4.06972980499, pi/2],
#                 [8.7332868576, 3.58700752258, pi/2],
#                 [8.39705371857, 3.49016475677, pi/2],
#                 [8.46412658691, 4.09582805634, pi/2],
#                 [8.42993354797, 4.06738042831, pi/2],
#                 [8.25302028656, 3.49167132378, pi/2],
#                 [5.10396099091, 3.40945029259, pi/2],
#                 [4.82347631454, 4.47493648529, pi/2],
#                 [5.62562417984, 4.33915901184, pi/2],
#                 [7.40439224243, 4.45074224472, pi/2]
#             ]
#         g = Goal_move(positions)
#         rospy.spin()

#     except rospy.ROSInterruptException:
#         print "program interrupted before completion"