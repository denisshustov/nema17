#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Twist
import tf.transformations
import tf

from actionlib_msgs.msg import GoalStatus

from std_msgs.msg import ByteMultiArray

from path_creator.srv import way_points_srv, way_points_srvRequest
from path_creator.srv import conturs_by_point_srv, conturs_by_point_srvRequest
import threading

import os
import sys
sys.path.append(os.path.join(sys.path[0], '../../libraries'))
from marker_lib import *

class Goal_move():

    def __init__(self):
        rospy.init_node('path_creator_goal_mover')
        rospy.on_shutdown(self.shutdown)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/ppp/cmd_vel', Twist, queue_size = 50)

        self.funAndBrushes_pub = rospy.Publisher("/funAndBrushes", ByteMultiArray, queue_size = 1)

        
        rospy.loginfo('Waiting for server...')
        self.client.wait_for_server(rospy.Duration(5))
        rospy.wait_for_service('contur_creator/get_by_xy')
        rospy.wait_for_service('path_creator/get_by_id')
        
    def get_conturs_by_xy(self,x,y):
        try:
            rospy.loginfo("try call service /contur_creator/get_by_xy")

            get_by_xy = rospy.ServiceProxy('/contur_creator/get_by_xy', conturs_by_point_srv)
            rqt = conturs_by_point_srvRequest(Point(x,y,0))
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

    def clean(self, is_turn_on):
        bma = ByteMultiArray()
        if is_turn_on:
            bma.data = [1,1,1]
        else:
            bma.data = [0,0,0]
        self.funAndBrushes_pub.publish(bma)

    def process(self, goals):
        self.clean(False)

        i=0
        for g in goals:
            goal = MoveBaseGoal()
            goal.target_pose.pose = g
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()

            self.markers = Marker_lib('viz_current_goal_markers', [[g.x,g.y,g.z]])

            rospy.loginfo('Sending goal to action server: %s' % goal)
            self.client.send_goal(goal)

    # PENDING = 0
    # ACTIVE = 1
    # DONE = 2


            done_condition = threading.Condition()

            #-----------------------------------------
            finished_within_time = False
            timeout = rospy.Duration(10)
            timeout_time = rospy.get_rostime() + timeout
            loop_period = rospy.Duration(0.1)
            with done_condition:
                while not rospy.is_shutdown():
                    time_left = timeout_time - rospy.get_rostime()
                    if timeout > rospy.Duration(0.0) and time_left <= rospy.Duration(0.0):
                        break

                    if self.client.simple_state == 2:
                        break

                    if time_left > loop_period or timeout == rospy.Duration():
                        time_left = loop_period

                    self.markers.publish()

                    done_condition.wait(time_left.to_sec())
            finished_within_time = self.client.simple_state == 2
            #-----------------------------------------

            if not finished_within_time:
                self.client.cancel_goal()
                rospy.loginfo("Timed out achieving goal {}. x ={} y={}".format(i, g.x, g.y))
                rospy.loginfo('Result received. Action state is %s' % self.client.get_state())
                rospy.loginfo('Goal status message is: {}'.format(self.client.get_goal_status_text()))
            else:
                state = self.client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    rospy.loginfo('Result received. Action state is %s' % self.client.get_state())
                    rospy.loginfo('Goal status message is %s' % self.client.get_goal_status_text())
            i+=1

if __name__ == '__main__':
    try:
        
        g = Goal_move()
        contur_error = True
        tf_listener = tf.TransformListener()

        while contur_error:
            try:
                (trans,rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except Exception as ex:
                rospy.loginfo("Transform map => base_link NOT FOUND!!! ERROR:{}".format(ex))
                rospy.sleep(1)
                continue

            current_contur = g.get_conturs_by_xy(trans[0], trans[1])
            
            contur_error = current_contur == None or current_contur.error_code != ''
            if contur_error:
                rospy.loginfo(current_contur.error_code)
                rospy.sleep(1)
            else:
                break

        goals = g.get_path(current_contur.contur_id)
        g.process(goals.points)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion")