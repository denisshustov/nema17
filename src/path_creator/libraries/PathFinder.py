import cv2
import numpy as np
import math


class PathFinder:
    
    def __init__(self, contours, image_hw, \
        GRID_SIZE = 10, border_in = 5, neibor_distance = 8, \
        start_point=None,debug_mode=False):

        if len(image_hw) == 0:
            raise Exception('image_hw has 0 shape')
        if len(contours)==0:
            raise Exception('contours len is 0')
        if GRID_SIZE==0:
            raise Exception('GRID_SIZE is 0!')
        
        self.debug_mode = debug_mode
        self.start_point = start_point
        self.neibor_distance = neibor_distance
        self.grid_points = []
        self.mounting_points = []
        self.path_points = []

        self.GRID_SIZE = GRID_SIZE
        self.border_in = border_in
        self.image_hw = image_hw
        self.contours = contours
        self.array_of_contours = np.empty(shape=[0, 2],dtype=int)
        for c in contours:
            self.array_of_contours = np.concatenate((self.array_of_contours,np.array(c).reshape(-1,2)))
    
    def line_intersection(self, line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0 :#or div >= max_w or div >= max_h:
            return None

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return int(x), int(y)

    def dist(self, p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        
    def getNeibors(self, point, array_fnd_in, threshold):
        result = []

        for x, y in array_fnd_in:
            if point[0]!=x or point[1]!=y:
                distance = self.dist(point, (x, y))
                if distance>=0 and distance<=threshold and not (x, y, distance) in result:
                    result.append((x, y, distance))
        result = sorted(result, key=lambda s: s[2])
        return result

    def get_relevant_points(self, neibors,un_x, ok_arr,un_x_ptr=None):
        result = []
        for n in neibors:
            if ((un_x_ptr==None and n[0] in un_x) or (n[0] in un_x[:un_x_ptr])) and \
            not (n[0],n[1]) in ok_arr:
                result.append(n)
        return result
    
    def show_grid(self, img):
        if not self.debug_mode:
            raise Exception('debug_mode should be true')

        for g in self.grid_points:
            cv2.line(img, g[0], g[1], (255, 0, 0), 1, 1)
    
    def show_mounting_point(self, img):
        if not self.debug_mode:
            raise Exception('debug_mode should be true')

        for p in self.mounting_points:
            cv2.circle(img, (p[0], p[1]),1, (0,0,255), -1)
    
    def show_path_point(self, img):
        if len(self.path_points)==0:
            return
        x=self.path_points[0][0]
        y=self.path_points[0][1]

        i=1
        while i<len(self.path_points):
            x_next=self.path_points[i][0]
            y_next=self.path_points[i][1]        
            cv2.line(img, (x,y), (x_next,y_next), (0, 50, 255), 1, 1)
            x=x_next
            y=y_next
            i+=1

    def get_in_points(self):
        mounting_points = []
        y_min = min(self.array_of_contours[:,1:])[0]
        y_max = max(self.array_of_contours[:,1:])[0]+self.GRID_SIZE

        x_min = min(self.array_of_contours[:,:1])[0]
        x_max = max(self.array_of_contours[:,:1])[0]+self.GRID_SIZE
        prev = (None,None)

        height = self.image_hw[0]
        width  = self.image_hw[1]
        for x in range(x_min, x_max, self.GRID_SIZE):
            if self.debug_mode:
                self.grid_points.append(((x, 0), (x, height)))

            for y in range(y_min, y_max, self.GRID_SIZE):
                if self.debug_mode:
                    self.grid_points.append(((0, y), (width, y)))

                intersect_point = self.line_intersection(((x, 0), (x, height)),((0, y), (width, y)))
                if intersect_point:
                    for c in self.contours:
                        is_in = cv2.pointPolygonTest(c, (intersect_point[0], intersect_point[1]), True)
                        if is_in >= self.border_in:
                            mounting_points.append((intersect_point[0], intersect_point[1]))
        return mounting_points

    def get_start_point(self, points):
        if self.start_point == None:
            return points[0]
        else:
            neibors = self.getNeibors(self.start_point, points, self.neibor_distance*100)
            return neibors[0]
            # return points[0]

    # def route2_get_next(self):
    #     if self.route2_state == 'BOTTOM':


    def get_route2(self):
        self.mounting_points = self.get_in_points()

        if len(self.mounting_points)==0:
            return []

        goto_to_wall = False
        xy = self.get_start_point(self.mounting_points ) #self.mounting_points[0]
        i=0
        step_finished = None
        #if not goto_to_wall:
        while True:
            neibors = self.getNeibors(xy,self.mounting_points,self.neibor_distance)
            x_grid_slice = np.unique(np.array(self.mounting_points)[:,:1], axis=0)
            relevant_points = self.get_relevant_points(neibors,x_grid_slice, self.path_points)
            min_distance = np.min(np.unique(np.array(relevant_points)[:,2:3], axis=0).flatten())    
            round_neibors = [r for r in relevant_points if r[2]==min_distance]
            relative_round_neibors = round_neibors-np.array(xy)
            index_left = [i for i,r in enumerate(relative_round_neibors) if r[0]<0 and r[1]==0]
            index_top = [i for i,r in enumerate(relative_round_neibors) if r[0]==0 and r[1]>0]
            index_right = [i for i,r in enumerate(relative_round_neibors) if r[0]>0 and r[1]==0]
            index_bottom = [i for i,r in enumerate(relative_round_neibors) if r[0]==0 and r[1]<0]

#-,+ | 0,+ | +,+
#-,0 |     | +,0
#-,- | 0,- | +,-
            if step_finished == None or step_finished == 'RIGHT':
                if len(index_bottom)>0:
                    self.path_points.append(round_neibors[index_bottom[0]])
                    xy = round_neibors[index_bottom[0]]
                else:
                    step_finished = 'BOTTOM'

            if step_finished == 'BOTTOM':
                if  len(index_left)>0:
                    self.path_points.append(round_neibors[index_left[0]])
                    xy = round_neibors[index_left[0]]
                else:
                    step_finished = 'LEFT'

            if step_finished == 'LEFT':
                if len(index_top)>0:
                    self.path_points.append(round_neibors[index_top[0]])
                    xy = round_neibors[index_top[0]]
                else:
                    step_finished = 'TOP'

            if step_finished == 'TOP':
                if len(index_right)>0:
                    self.path_points.append(round_neibors[index_right[0]])
                    xy = round_neibors[index_right[0]]
                else:
                    step_finished = 'RIGHT'
            if i>2000:
                break
            i+=1
        return None

    def get_route(self):
        self.mounting_points = self.get_in_points()

        if len(self.mounting_points)==0:
            return []
            #raise Exception('int_points is empty, something go wrong')
        #OK!!!!!!!!!!!!!!!!!!
        x_grid_slice = np.unique(np.array(self.mounting_points)[:,:1], axis=0)
        x_grid_ptr = 0

        k=0
        xy = self.get_start_point(self.mounting_points ) #self.mounting_points[0]
        
        while True:
            x = xy[0]
            y = xy[1]

            neibors = self.getNeibors(xy,self.mounting_points,self.neibor_distance)
            if self.start_point == None:
                relevant_points = self.get_relevant_points(neibors,x_grid_slice, self.path_points, x_grid_ptr)
            else:
                relevant_points = self.get_relevant_points(neibors,x_grid_slice, self.path_points)
                self.start_point = None

            if len(relevant_points) > 0:
                relevant_points_fst = sorted(relevant_points, key=lambda s: s[2])[0]
                if not relevant_points_fst in self.path_points:
                    self.path_points.append((relevant_points_fst[0],relevant_points_fst[1]))
                    xy = relevant_points_fst
                else:
                    x_grid_ptr+=1
                    if len(x_grid_slice)<=x_grid_ptr:
                        break
            else:
                all_neibors = [n for n in self.getNeibors(xy,self.mounting_points,100500) if not (n[0],n[1]) in self.path_points]
                if len(all_neibors)>0: # and all_neibors[0][2] > self.neibor_distance:
                    #just to first
                    self.path_points.append((all_neibors[0][0],all_neibors[0][1]))
                    xy = all_neibors[0]
                else:
                    x_grid_ptr+=1
                    if len(x_grid_slice)<=x_grid_ptr:
                        break

            # if k>1117111:
            #     break
            k+=1
        return self.path_points