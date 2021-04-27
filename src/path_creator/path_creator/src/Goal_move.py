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
from actionlib_msgs.msg import GoalStatus

from std_msgs.msg import ByteMultiArray

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

    def __init__(self, goal_list):
        rospy.init_node('goal_move')
        rospy.on_shutdown(self.shutdown)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/ppp/cmd_vel', Twist, queue_size = 50)

        self.funAndBrushes_pub = rospy.Publisher("/funAndBrushes", ByteMultiArray, queue_size = 1)

        self.goal_list = goal_list
        self.current_goal_index = 0

        print 'Waiting for server...'
        self.client.wait_for_server(rospy.Duration(5))
        
        self.go_to_goal()

    def shutdown(self):
        bma = ByteMultiArray()
        bma.data = [0,0,0]
        self.funAndBrushes_pub.publish(bma)
        rospy.sleep(1)

        rospy.loginfo("Stopping the robot...")
        self.client.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    
    def choose_next_goal(self):
        if self.current_goal_index + 1 < len(self.goal_list):
            rospy.loginfo("Go to next goal {}".format(self.current_goal_index))
            self.current_goal_index += 1
            self.go_to_goal()
        else:
            rospy.loginfo("Goals finished")
            rospy.signal_shutdown("Shutting down...")

    def go_to_goal(self):
        bma = ByteMultiArray()
        bma.data = [1,1,1]
        self.funAndBrushes_pub.publish(bma)

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

        finished_within_time = self.client.wait_for_result(rospy.Duration(10)) 

        if not finished_within_time:
            self.client.cancel_goal()
            rospy.loginfo("Timed out achieving goal {}".format(self.current_goal_index))
            print 'Result received. Action state is %s' % self.client.get_state()
            print('Goal status message is: {}'.format(self.client.get_goal_status_text()))

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