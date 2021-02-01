#!/usr/bin/env python2

import roslib 
import rospy
import actionlib
import geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import tf.transformations
import time
from math import radians, pi
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class Goal_move():
    STATUSES = {
        0:'The goal has yet to be processed by the action server',
        1:'The goal is currently being processed by the action server',
        2:'The goal received a cancel request after it started executing and has since completed its execution (Terminal State)',
        3:'The goal was achieved successfully by the action server (Terminal State)',
        4:'The goal was aborted during execution by the action server due to some failure (Terminal State)',
        5:'The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)',
        6:'The goal received a cancel request after it started executing and has not yet completed execution',
        7:'The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled',
        8:'The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)',
        9:'An action client can determine that a goal is LOST. This should not be sent over the wire by an action server'
    }

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
        
        print 'Publish markers'
        self.marker_pub.publish(self.markerArray)

    def __init__(self, goal_list):
        rospy.init_node('goal_move')
        rospy.on_shutdown(self.shutdown)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/ppp/cmd_vel', Twist, queue_size = 50)
        self.marker_pub = rospy.Publisher('visualization_marker', MarkerArray, queue_size = 1)

        self.markerArray = MarkerArray()
        self.goal_list = goal_list
        self.current_goal_index = 0

        self.init_markers()
        print 'Waiting for server...'
        self.client.wait_for_server(rospy.Duration(5))
        
        self.go_to_goal()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.client.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    
    def choose_next_goal(self):
        if self.current_goal_index + 1 < len(self.goal_list):
            rospy.loginfo("Go to next goal")
            self.current_goal_index += 1
            self.go_to_goal()
        else:
            rospy.loginfo("Goals finished")
            rospy.signal_shutdown("Shutting down...")

    def go_to_goal(self):

        pose = geometry_msgs.msg.Pose()
        pose.position.x = self.goal_list[self.current_goal_index][0]
        pose.position.y = self.goal_list[self.current_goal_index][1]
        pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0, 0, self.goal_list[self.current_goal_index][2])
        pose.orientation = geometry_msgs.msg.Quaternion(*q)

        goal = MoveBaseGoal()
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        print 'Sending goal to action server: %s' % goal
        self.client.send_goal(goal)
        self.marker_pub.publish(self.markerArray)

        finished_within_time = self.client.wait_for_result(rospy.Duration(30)) 

        if not finished_within_time:
            self.client.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            print 'Result received. Action state is %s' % self.client.get_state()
            print 'Goal status message is %s' % self.client.get_goal_status_text()
            self.choose_next_goal()
        else:
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                print 'Result received. Action state is %s' % self.client.get_state()
                print 'Goal status message is %s' % self.client.get_goal_status_text()
                self.choose_next_goal()

if __name__ == '__main__':
    try:
        positions = [
                [8.77073478699, 4.06972980499, pi/2],
                [8.7332868576, 3.58700752258, pi/2],
                [8.39705371857, 3.49016475677, pi/2],
                [8.46412658691, 4.09582805634, pi/2],
                [8.42993354797, 4.06738042831, pi/2],
                [8.25302028656, 3.49167132378, pi/2],
                [5.10396099091, 3.40945029259, pi/2],
                [4.82347631454, 4.47493648529, pi/2],
                [5.62562417984, 4.33915901184, pi/2],
                [7.40439224243, 4.45074224472, pi/2]
            ]
        g = Goal_move(positions)
        rospy.spin()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"