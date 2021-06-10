#!/usr/bin/env python3

import rospy
import os
import sys
sys.path.append(os.path.join(sys.path[0], '../../libraries'))
from WayPoint import *
from path_helper import *

class Path_Visualizer:
    def __init__(self):
        rospy.init_node("path_visualizer_node")
        
        rospy.wait_for_service('contur_creator/get_conturs')
        rospy.wait_for_service('contur_creator/get_by_xy')
        rospy.wait_for_service('path_creator/get_by_id')

        rospy.loginfo("path_visualizer_node Starting...")
        self.rate = rospy.get_param('~rate',10.0)
    
    def process(self):
        way_points = get_way_points()
        if way_points == None:
            return

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            for w in way_points:
                w.display()

            r.sleep()

if __name__ == '__main__':
    v = Path_Visualizer()
    v.process()
