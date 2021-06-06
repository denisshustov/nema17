#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf

import os
import sys
import numpy as np

sys.path.append(os.path.join(sys.path[0], '../../libraries'))
from Conturs import *

from PathFinder import *

# from WayPoint import *

from path_creator.srv import way_points_srv, way_points_srvResponse, conturs_srvResponse, conturs_srv
from path_creator.srv import conturs_by_point_srv, conturs_by_point_srvResponse

from map_contur_msg.msg import map_contur_msg

class Path_Creator:
    def __init__(self):
        rospy.init_node("path_creator")
        rospy.Subscriber("/map", OccupancyGrid, self.callback)
        self.map = None
        self.array_map = []

        self.cnt_inst = None
        self.robot_diametr = 0.3
        self.robot_center_pixel_x = 10
        self.robot_center_pixel_y = 10        
        # self.way_points = []
        self.covered_points = []
        self.conutrs = []
        self.find_conutrs_in_progress = False
        self.find_path_in_progress = None

        self.srv = rospy.Service('path_creator/get_by_id', way_points_srv, self.get_path_by_id)

        self.srv1 = rospy.Service('contur_creator/get_conturs', conturs_srv, self.get_conturs)
        self.srv2 = rospy.Service('contur_creator/get_by_id', conturs_srv, self.get_contur_by_id)
        self.srv2 = rospy.Service('contur_creator/get_by_xy', conturs_by_point_srv, self.get_contur_by_xy)
        self.srv3 = rospy.Service('contur_creator/get_by_next_by_id', conturs_srv, self.get_by_next_by_id)

        # self.tf_listener = tf.TransformListener()
        rospy.loginfo("path_creator Starting...")
        self.rate = rospy.get_param('~rate',100.0)
        
        self.merge = []
        self.is_merged = False
        for i in range(0, 10):
            try:
                m = eval(rospy.get_param('~merge_{}'.format(i),'[]'))
                if len(m)>0:
                    self.merge.append(m)
            except Exception: pass

        rospy.spin()

    def callback(self, data):
        self.map = data

#--------------path-----------
    def get_path_by_id(self, request):
        if len(request.contur_id)==0:
            return way_points_srvResponse(error_code="contur_id_IS_EMPTY")
        if self.find_path_in_progress == request.contur_id:
            return way_points_srvResponse(error_code="FIND_PATH_FOR_CURRENT_ID_IN_PROGRESS")

        if len(self.conutrs) == 0:
            self.conutrs = self._get_conturs()
        if len(self.conutrs) == 0:
            return conturs_srvResponse(error_code="CONTURS_NOT_FOUND")
        
        contur = None
        for c in self.conutrs:
            if c.id == request.contur_id:
               contur = c
               break
        if contur == None:
            return way_points_srvResponse(error_code="contur_id_NOT_FOUND")

        result = self._get_path(contur, request.x, request.y)            
        return way_points_srvResponse(points = result, contur_id = request.contur_id)
    
    def _get_path(self, contur, current_position_x = None, current_position_y = None):
        start_point = None

        try:
            if current_position_x != None and current_position_y != None:
                start_point = { 0 : current_position_x, 1 : current_position_y }
            self.find_path_in_progress = contur.id
            pth = PathFinder(contur.contur, self.array_map.shape, 5, 3, start_point=start_point, debug_mode=False)
            self.covered_points = pth.get_route()
            points = self.correct_points(self.covered_points)
        finally:
            self.find_path_in_progress = None
        return points

#--------------path-----------
    
    def save_array_to_file(self, arr):
        np.savetxt('/home/den/catkin_ws/src/path_creator/test/test1.txt', arr, fmt='%d')
    def load_array_to_file(self):
        return np.loadtxt('/home/den/catkin_ws/src/path_creator/test/test1.txt', dtype=int)

    def map_to_array(self):
        result = np.reshape(self.map.data, (-1, self.map.info.width))

        np.place(result, result ==255, -100500)
        np.place(result, result ==0, 255)
        np.place(result, result ==-1, 0)
        np.place(result, result ==100, 0)
        np.place(result, result ==-100500, 0)
        result = np.uint8(result)
        #array_2d_rgb = backtorgb = cv2.cvtColor(result,cv2.COLOR_GRAY2RGB)
        return result

#-------------contur------------------

    def get_contur_by_xy(self, request):
        if self.map == None:
            return conturs_by_point_srvResponse(error_code="MAP_NOT_FOUND")

        if request.point == None:
            return conturs_by_point_srvResponse(error_code="FIND_CONTURS_IN_PROGRESS")
        
        if len(self.conutrs) == 0:
            self.conutrs = self._get_conturs()
        if len(self.conutrs) == 0:
            return conturs_by_point_srvResponse(error_code="CONTURS_NOT_FOUND")
        
        # (trans,rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))

        xx = int((request.point.x/self.map.info.resolution)+(self.map.info.width/2))
        yy = int((request.point.y/self.map.info.resolution)+(self.map.info.height/2))
        
        contur_by_point = self.cnt_inst.get_contur_by_coord(xx,yy)
        if contur_by_point == None:
            return conturs_by_point_srvResponse(error_code="CONTURS_NOT_FOUND_BY_POSITION")

        cor_con = self.correct_points(contur_by_point.corrected_contur)
        return conturs_by_point_srvResponse(contur_id = contur_by_point.id, conturs = [map_contur_msg(points = cor_con)])

    def correct_points(self, points):
        result = []
        for p in points:
            result.append(Point((p[0]*self.map.info.resolution)+self.map.info.origin.position.x, \
                (p[1]*self.map.info.resolution)+self.map.info.origin.position.y,0))
        return result


    def get_by_next_by_id(self, request):
        if self.find_conutrs_in_progress:
            return way_points_srvResponse(error_code="FIND_CONTURS_IN_PROGRESS")
        if len(request.contur_id)==0:
            return conturs_srvResponse(error_code="contur_id_IS_EMPTY")

        if len(self.conutrs) == 0:
            self.conutrs = self._get_conturs()
        if len(self.conutrs) == 0:
            return conturs_srvResponse(error_code="CONTURS_NOT_FOUND")

        cor_con = self._get_contur_by_id_with_points(request.contur_id)
        if cor_con == None:
            return conturs_srvResponse(error_code="contur_id_NOT_FOUND")
        if len(cor_con.children)==0:
            return conturs_srvResponse(error_code="children_by_id_NOT_FOUND")

        result = []
        for r in cor_con.children:
            result.append(map_contur_msg(r.id,[]))

        return conturs_srvResponse(conturs = result)

    def _get_contur_by_id(self, id):
        contur = self._get_contur_by_id(id)
        return self.correct_points(contur.corrected_contur)

    def _get_contur_by_id_with_points(self, id):
        for contur in self.conutrs:
            if contur.id == id:
                return contur
        return None

    def get_contur_by_id(self, request):
        if self.find_conutrs_in_progress:
            return way_points_srvResponse(error_code="FIND_CONTURS_IN_PROGRESS")
        if len(request.contur_id)==0:
            return conturs_srvResponse(error_code="contur_id_IS_EMPTY")

        if len(self.conutrs) == 0:
            self.conutrs = self._get_conturs()
        if len(self.conutrs) == 0:
            return conturs_srvResponse(error_code="CONTURS_NOT_FOUND")
        
        cor_con = self._get_contur_by_id(request.contur_id)
        if cor_con == None:
            return conturs_srvResponse(error_code="contur_id_NOT_FOUND")
           
        return conturs_srvResponse(conturs = [map_contur_msg(contur_id = request.contur_id, points = cor_con)])

    def get_conturs(self, request):
        if self.find_conutrs_in_progress:
            return way_points_srvResponse(error_code="FIND_CONTURS_IN_PROGRESS")
        if len(self.conutrs) == 0:
            self.conutrs = self._get_conturs()
        if len(self.conutrs) == 0:
            return conturs_srvResponse(error_code="CONTURS_NOT_FOUND")
        
        all_conturs = []
        for cc in self.conutrs:
            cor_con = self.correct_points(cc.corrected_contur)
            all_conturs.append(map_contur_msg(contur_id = cc.id, points = cor_con))
        return conturs_srvResponse(conturs = all_conturs)

    def _get_conturs(self):
        conutrs = []
        if self.map != None and not self.find_conutrs_in_progress:
            if len(self.conutrs)>0:
                self.find_conutrs_in_progress = False
                return self.conutrs

            self.find_conutrs_in_progress = True
            self.array_map = self.map_to_array()
            # self.save_array_to_file(self.array_map)

            self.cnt_inst = Conturs(self.array_map)
            conutrs = self.cnt_inst.get_conturs(skip_area_less_than = 40)
            
            if len(conutrs)==0:
                self.find_conutrs_in_progress = False
                return []

            if not self.is_merged and len(self.merge)>0:
                for m in self.merge:
                    new_contur = self.cnt_inst.merge(m)
                self.is_merged = True

            inter = self.cnt_inst.get_intersections(5)

            self.find_conutrs_in_progress = False
            return conutrs
        return []

#-------------contur------------------

if __name__ == '__main__':
    c = Path_Creator()
    # c.go()
