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

    def init_markers(self):
        markerArray = MarkerArray()
        publisher = rospy.Publisher('visualization_marker', MarkerArray, queue_size = 50)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 8.77073478699
        marker.pose.position.y = 4.06972980499
        marker.pose.position.z = 0
        
        markerArray.markers.append(marker)
        publisher.publish(markerArray)
        print 'publishe marker'

        rospy.sleep(0.01)

    def __init__(self, goal_list):
        rospy.init_node('goal_move')
        rospy.on_shutdown(self.shutdown)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/ppp/cmd_vel', Twist, queue_size = 50)

        self.goal_list = goal_list
        self.current_goal_index = 0

        print 'Waiting for server...'
        self.client.wait_for_server(rospy.Duration(5))
        
        self.init_markers()
        #self.go_to_goal()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.client.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def callback_active(self):
        #rospy.loginfo("Goal pose "+str(self.current_goal_index)+" is now being processed by the Action Server...")
        pass

    def callback_feedback(self, feedback):
        #rospy.loginfo("Feedback for goal pose "+str(self.current_goal_index)+" received." + str(feedback))
        pass

#http://docs.ros.org/en/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html

    def callback_done(self, status, result):
        rospy.loginfo("status={}, result={}".format(status, result))

        if status != 3:
            rospy.loginfo("Goal pose "+str(self.current_goal_index)+" received a cancel request after it started executing, completed execution!")
        elif status == 3:
            rospy.loginfo("Goal pose "+str(self.current_goal_index)+" reached")
        
        rospy.loginfo("STATUS = {} ".format(self.client.get_goal_status_text()))
        
        self.choose_next_goal()
    
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
        self.client.send_goal(goal, self.callback_done, self.callback_active, self.callback_feedback)

        finished_within_time = self.client.wait_for_result(rospy.Duration(30)) 

        if not finished_within_time:
            #self.client.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            #self.choose_next_goal()
        # else:
        #     # We made it!
        #     state = self.client.get_state()
        #     if state == GoalStatus.SUCCEEDED:
        #         rospy.loginfo("Goal succeeded!")

        # print 'Result received. Action state is %s' % self.client.get_state()
        # print 'Goal status message is %s' % self.client.get_goal_status_text()


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

# #!/usr/bin/env python2

# #import requests
# import roslib;# roslib.load_manifest('rbx1_nav')
# import rospy
# import actionlib
# from actionlib_msgs.msg import *
# from geometry_msgs.msg import Pose, Point, Quaternion, Twist
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from tf.transformations import quaternion_from_euler
# from visualization_msgs.msg import Marker
# from math import radians, pi

# class Goal_move():
#     def __init__(self):
#         rospy.init_node('nav_test', anonymous=False)
        
#         rospy.on_shutdown(self.shutdown)
        
#         # How big is the square we want the robot to navigate?
#         points = [[7.40554666519,3.98127555847], [7.35754680634,4.37160396576]]#rospy.get_param("~square_size", 1.0) # meters
        
#         # Create a list to hold the target quaternions (orientations)
#         quaternions = list()
        
#         # First define the corner orientations as Euler angles
#         euler_angles = (pi/2, pi, 3*pi/2, 0)
        
#         # Then convert the angles to quaternions
#         for angle in euler_angles:
#             q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
#             q = Quaternion(*q_angle)
#             quaternions.append(q)
        
#         # Create a list to hold the waypoint poses
#         waypoints = list()

#         for point in points:
#             pose = Pose()
#             pose.position.x = point[0]
#             pose.position.y = point[1]
#             pose.position.z = 0.0
#             pose.orientation = quaternions[3]
#             waypoints.append(pose)

#         # Initialize the visualization markers for RViz
#         self.init_markers()
        
#         # Set a visualization marker at each waypoint        
#         for waypoint in waypoints:
#             p = Point()
#             p = waypoint.position
#             self.markers.points.append(p)
            
#         # Publisher to manually control the robot (e.g. to stop it)
#         self.cmd_vel_pub = rospy.Publisher('/ppp/cmd_vel', Twist, queue_size = 50)
        
#         # Subscribe to the move_base action server
#         self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
#         rospy.loginfo("Waiting for move_base action server...")
        
#         # Wait 60 seconds for the action server to become available
#         self.move_base.wait_for_server(rospy.Duration(5))
        
#         rospy.loginfo("Connected to move base server")
#         rospy.loginfo("Starting navigation test")
        
#         # Initialize a counter to track waypoints
#         i = 0
        
#         # Cycle through the four waypoints
#         while i < 2 and not rospy.is_shutdown():
#             # Update the marker display
#             self.marker_pub.publish(self.markers)
            
#             # Intialize the waypoint goal
#             goal = MoveBaseGoal()
#             goal.target_pose.header.frame_id = 'map'
#             goal.target_pose.header.stamp = rospy.Time.now()
#             goal.target_pose.pose = waypoints[i]
            
#             # Start the robot moving toward the goal
#             self.move(goal)
            
#             i += 1
        
#     def move(self, goal):
#             self.move_base.send_goal(goal)
#             finished_within_time = self.move_base.wait_for_result(rospy.Duration(30)) 
            
#             # If we don't get there in time, abort the goal
#             if not finished_within_time:
#                 self.move_base.cancel_goal()
#                 rospy.loginfo("Timed out achieving goal")
#             else:
#                 # We made it!
#                 state = self.move_base.get_state()
#                 if state == GoalStatus.SUCCEEDED:
#                     rospy.loginfo("Goal succeeded!")
                    

#     def shutdown(self):
#         rospy.loginfo("Stopping the robot...")
#         # Cancel any active goals
#         self.move_base.cancel_goal()
#         rospy.sleep(2)
#         # Stop the robot
#         self.cmd_vel_pub.publish(Twist())
#         rospy.sleep(1)

# if __name__ == '__main__':
#     try:
#         Goal_move()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Navigation test finished.")