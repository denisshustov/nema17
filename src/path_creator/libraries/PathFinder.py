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
            cv2.line(img, (x,y), (x_next,y_next), (255, 50, 255), 2, 1)
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
        if self.start_point == None or (self.start_point[0]==0 and self.start_point[1]==0):
            return points[0]
        else:
            neibors = self.getNeibors(self.start_point, points, self.neibor_distance*100)
            return neibors[0]

    def add_point(self, round_neibors, step_to, without_add = False):
        new_pos=round_neibors[step_to[0]]
        if not without_add:
            self.path_points.append((new_pos[0],new_pos[1]))
        return (new_pos[0],new_pos[1])

    def get_nearest_wall(self, mounting_points, x, y):
        left = min(np.array(mounting_points)[:,0:1])#x
        right = max(np.array(mounting_points)[:,0:1])#x

        top = min(np.array(mounting_points)[:,1:])#y
        bottom = max(np.array(mounting_points)[:,1:])#y

        min_directons = np.array([ \
            self.dist([x, bottom],[x,y]), \
            self.dist([x, top],[x,y]),    \
            self.dist([left, y],[x,y]),   \
            self.dist([right, y],[x,y])])

        return ['LEFT','RIGHT', 'TOP','BOTTOM'][min_directons.argmin()]

    def get_route2(self, is_clockwise = True):
        self.mounting_points = self.get_in_points()

        if len(self.mounting_points)==0:
            return []

        xy = self.get_start_point(self.mounting_points ) #self.mounting_points[0]
        x = xy[0]
        y = xy[1]
        self.add_point([[x,y]], [0])
        i=0
        goto_direct = self.get_nearest_wall(self.mounting_points,x,y)
        direct_priority = {
            'BOTTOM':[[2,2], [2,1], [2,0]],
            'TOP':   [[0,0], [0,1], [0,2]],
            'LEFT':  [[2,0], [1,0], [0,0]],
            'RIGHT': [[0,1], [1,2], [0,2]]
        }
        direct_next = {
            'BOTTOM':'LEFT',
            'TOP':   'RIGHT',
            'LEFT':  'TOP',
            'RIGHT': 'BOTTOM'
        }
        is_no_way = True

        while True:
            neibors = self.getNeibors((x,y),self.mounting_points,self.neibor_distance)
            x_grid_slice = np.unique(np.array(self.mounting_points)[:,:1], axis=0)
            relevant_points = self.get_relevant_points(neibors,x_grid_slice, self.path_points)
            if len(relevant_points)==0:
                q=123
                break
            sorted1 = sorted(relevant_points, key=lambda s: s[2])
            round_neibors = [[r[0],r[1]] for r in sorted1]

            relative_round_neibors = round_neibors-np.array([x,y])
            availabel_points = [[ [i for i,r in enumerate(relative_round_neibors) if r[0]<0 and r[1]<0], [i for i,r in enumerate(relative_round_neibors) if r[0]==0 and r[1]<0],[i for i,r in enumerate(relative_round_neibors) if r[0]>0 and r[1]<0]], \
                    [[i for i,r in enumerate(relative_round_neibors) if r[0]<0 and r[1]==0],[],[i for i,r in enumerate(relative_round_neibors) if r[0]>0 and r[1]==0]], \
                    [[i for i,r in enumerate(relative_round_neibors) if r[0]<0 and r[1]>0],[i for i,r in enumerate(relative_round_neibors) if r[0]==0 and r[1]>0], [i for i,r in enumerate(relative_round_neibors) if r[0]>0 and r[1]>0]]]

            print('{}.'.format(goto_direct))
            for d in direct_priority[goto_direct]:
                ap = availabel_points[d[0]][d[1]]
                if len(ap)>0:
                    is_no_way = False
                    (x,y) = self.add_point(round_neibors, ap)
                    break
                is_no_way = True
            if is_no_way:
                goto_direct = direct_next[goto_direct]
#decart coordinates
#-,+ | 0,+ | +,+
#-,0 |     | +,0
#-,- | 0,- | +,-

#array coordinates
#-,- | 0,- | +,-
#-,0 |     | +,0
#-,+ | 0,+ | +,+
            # if i>50:
            #     break
            
            i+=1
        return self.path_points

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