#!/usr/bin/env python3

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
from nav_msgs.msg import Odometry

from std_msgs.msg import ByteMultiArray

from path_creator.srv import way_points_srv, way_points_srvRequest
from path_creator.srv import conturs_by_point_srv, conturs_by_point_srvRequest


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

    def __init__(self):
        rospy.init_node('path_creator_goal_mover')
        rospy.on_shutdown(self.shutdown)
        
        # self.goal_list = []
        self.current_goal_index = 0        
        self.pose = None

        odom_sub = rospy.Subscriber('/odom', Odometry, self.get_odometry)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/ppp/cmd_vel', Twist, queue_size = 50)

        self.funAndBrushes_pub = rospy.Publisher("/funAndBrushes", ByteMultiArray, queue_size = 1)

        
        rospy.loginfo('Waiting for server...')
        self.client.wait_for_server(rospy.Duration(5))
        rospy.wait_for_service('contur_creator/get_by_xy')
        rospy.wait_for_service('path_creator/get_by_id')
        
    def get_odometry(self, msg):
        self.pose = msg.pose.pose

    def get_conturs_by_xy(self,x,y):
        try:
            rospy.loginfo("try call service /contur_creator/get_by_xy")

            get_by_xy = rospy.ServiceProxy('/contur_creator/get_by_xy', conturs_by_point_srv)
            rqt = conturs_by_point_srvRequest(x,y)
            #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            resp = get_by_xy(rqt)
            
            rospy.loginfo("call service /contur_creator/get_by_xy success")
            return resp
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

    def get_path(self, contur_id):
        try:
            rospy.loginfo("try call service /path_creator/get_by_id {}".format(contur_id))

            get_path = rospy.ServiceProxy('/path_creator/get_by_id', way_points_srv)
            rqt = way_points_srvRequest(contur_id)
            resp = get_path(rqt)
            rospy.loginfo("call service /path_creator/get_by_id {} success".format(contur_id))

            return resp
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

    def shutdown(self):
        self.clean(False)
        rospy.sleep(1)

        rospy.loginfo("Stopping the robot...")
        self.client.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    
    # def choose_next_goal(self):
    #     if self.current_goal_index + 1 < len(self.goal_list):
    #         rospy.loginfo("Go to next goal {}".format(self.current_goal_index))
    #         self.current_goal_index += 1
    #         self.go_to_goal()
    #     else:
    #         rospy.loginfo("Goals finished")
    #         rospy.signal_shutdown("Shutting down...")

    def clean(self, is_turn_on):
        bma = ByteMultiArray()
        if is_turn_on:
            bma.data = [1,1,1]
        else:
            bma.data = [0,0,0]
        self.funAndBrushes_pub.publish(bma)

    def go_to_goal(self, goals):
        self.clean(True)

        i=0
        for g in goals:
            pose = geometry_msgs.msg.Pose()
            pose.position.x = g[0]
            pose.position.y = g[1]
            # pose.position.x = self.goal_list[self.current_goal_index][0]
            # pose.position.y = self.goal_list[self.current_goal_index][1]
            pose.position.z = 0.0

            q = tf.transformations.quaternion_from_euler(0, 0, g[2])
            # q = tf.transformations.quaternion_from_euler(0, 0, self.goal_list[self.current_goal_index][2])
            pose.orientation = geometry_msgs.msg.Quaternion(*q)

            goal = MoveBaseGoal()
            goal.target_pose.pose = pose
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()

            rospy.loginfo('Sending goal to action server: %s' % goal)
            self.client.send_goal(goal)

            finished_within_time = self.client.wait_for_result(rospy.Duration(10)) 

            if not finished_within_time:
                self.client.cancel_goal()
                rospy.loginfo("Timed out achieving goal {}".format(self.current_goal_index))
                rospy.loginfo('Result received. Action state is %s' % self.client.get_state())
                rospy.loginfo('Goal status message is: {}'.format(self.client.get_goal_status_text()))

                # self.choose_next_goal()
            else:
                state = self.client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    rospy.loginfo('Result received. Action state is %s' % self.client.get_state())
                    rospy.loginfo('Goal status message is %s' % self.client.get_goal_status_text())
                    # self.choose_next_goal()

if __name__ == '__main__':
    try:
        # positions = [
        #         [8.77073478699, 4.06972980499, pi/2],
        #         [8.7332868576, 3.58700752258, pi/2],
        #         [8.39705371857, 3.49016475677, pi/2],
        #         [8.46412658691, 4.09582805634, pi/2],
        #         [8.42993354797, 4.06738042831, pi/2],
        #         [8.25302028656, 3.49167132378, pi/2],
        #         [5.10396099091, 3.40945029259, pi/2],
        #         [4.82347631454, 4.47493648529, pi/2],
        #         [5.62562417984, 4.33915901184, pi/2],
        #         [7.40439224243, 4.45074224472, pi/2]
        #     ]
        g = Goal_move()
        current_contur = g.get_conturs_by_xy(455,420)
        goals = g.get_path(current_contur.id)
        g.process(goals)

        g.go_to_goal()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion")