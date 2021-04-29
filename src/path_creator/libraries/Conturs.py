import cv2
import numpy as np
import os
import random
import sys
import math

from skimage.morphology import watershed
from skimage.feature import peak_local_max
from skimage import morphology
from scipy import ndimage
from skimage.measure import regionprops


class Contur:
    def __init__(self, contur, corrected_contur, id, labels):
        self.contur = contur
        self.corrected_contur = corrected_contur
        self.id = id
        self.labels = labels
        self.children = []
        self.parent = None
        self.is_processed = False

class Conturs:
    def __init__(self, image):
        self.conturs = []
        self.image = image
        self.labels = []
        
    def intersect2D(self, a, b, delta): #long runing
        result = []
        for a1 in a:
            for b1 in b:
                if abs(a1[0]-b1[0])<=delta and abs(a1[1]-b1[1])<=delta:
                    return True
        return False

    def merge(self, id1, id2):
        c1=None
        c2=None

        for c in self.conturs:
            if c.id == id1:
                c1 = c
            if c.id == id2:
                c2 = c
        if c1==None or c2==None:
            raise Exception('id1 or id1 not foind in conturs')

        mask = np.zeros(self.image.shape, dtype="uint8")
        for l in c1.labels:
            mask[self.labels == l] = 255
        for l in c2.labels:
            mask[self.labels == l] = 255
            
        cnt, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        corrected_conturs=[]
        for idx, val in enumerate(cnt):
            area = cv2.contourArea(val)
            if area>10:
                for idx1, val1 in enumerate(val):
                    for idx2, val2 in enumerate(val1):
                        corrected_conturs.append((cnt[idx][idx1][idx2][0],cnt[idx][idx1][idx2][1]))

        self.conturs.remove(c1)
        self.conturs.remove(c2)
        new_c = Contur(cnt,corrected_conturs, str(id1),c1.labels + c2.labels)

        self.conturs.append(new_c)

        return self.conturs

    def get_contur_by_coord(self, x,y):
        for cnt in self.conturs:
            is_in = cv2.pointPolygonTest(cnt.contur[0], (x,y), True)
            if is_in>0:
                return cnt
        return None

    def get_intersections(self, delta = 2, ignore_without_children = True):
        for cnt in self.conturs:
            for cnt1 in self.conturs:
                if cnt.id != cnt1.id:
                    inters = self.intersect2D(np.array(cnt.corrected_contur), \
                        np.array(cnt1.corrected_contur), delta)
                    if inters:
                        cnt.children.append(cnt1)
                        cnt1.parent = cnt
        i=0
        while i<len(self.conturs):
            if len(self.conturs[i].children)==0:
                del self.conturs[i]
            i+=1
        return self.conturs
    
    # TODO TEST THIS!!!
    # def get_intersections(self, delta = 2, ignore_without_children = True):
    #     i=0
    #     while i<len(self.conturs):
    #         cnt = self.conturs[i]
    #         j = i + 1
    #         while j<len(self.conturs):
    #             cnt1 = self.conturs[j]
    #             if cnt.id != cnt1.id:
    #                 inters = self.intersect2D(np.array(cnt.corrected_contur), \
    #                     np.array(cnt1.corrected_contur), delta)
    #                 if inters:
    #                     cnt.children.append(cnt1)
    #                     cnt1.parent = cnt
    #             j+=1
    #         i+=1

    #     i=0
    #     while i<len(self.conturs):
    #         if len(self.conturs[i].children)==0:
    #             del self.conturs[i]
    #         i+=1
    #     return self.conturs

    def set_unvisited(self):
        for c in self.conturs:
            c.is_processed = False

    def show(self, img):
        i=0
        current_contur = self.conturs[0]

        while True:
            current_contur.is_processed = True
            cv2.drawContours(img, current_contur.contur, -1, (0, 0, 255), 1, 1)
            props = regionprops(self.labels)
            for cc in current_contur.labels:
                current_prop = [p for p in props if p.label == cc][0]
                x = int(current_prop.centroid[1])
                y = int(current_prop.centroid[0])
                cv2.putText(img, current_contur.id, (x,y), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.3, \
                    color=(0,0,0),thickness=2)

            current_conturs = self.get_contur_in_order(current_contur)
            if current_conturs == None or current_conturs == []:
                break
            current_contur = current_conturs[0]
        self.set_unvisited()

    def find_loop(self, seq, subseq):
        n = len(seq)
        m = len(subseq)
        for i in range(n - m + 1):
            found = True
            for j in range(m):
                if seq[i + j] != subseq[j]:
                    found = False
                    break
            if found:
                yield i

    def get_contur_in_order(self, current, go_from = None, skip_sequence = [], \
        skip_sequence2 = [], hops = 0):

        if len(current.children)>0:
            for c in current.children:
                if not c.is_processed:
                    return [c]

            for c in current.children:
                if go_from == None or (go_from != None and go_from.id != c.id):

                        if go_from!=None:
                            skip_sequence.append(go_from.id)
                            skip_sequence.append(current.id)
                        if hops>15:
                            skip_sequence2.append(go_from.id)
                            skip_sequence2.append(current.id)
                        is_break = False
                        if hops>20:
                            for _ in self.find_loop(skip_sequence,skip_sequence2):
                                is_break=True
                                break
                            if is_break:
                                return []
                        hops+=1
                        
                        res = self.get_contur_in_order(c, current, \
                            skip_sequence = skip_sequence, skip_sequence2 = skip_sequence2,\
                            hops = hops)
                        if res == []:
                            return []
                        if res != None:
                            res.append(c)
                            res.append(current)
                            return res

        return None

    def get_conturs(self, skip_area_less_than = 10):
        distance = cv2.distanceTransform(self.image, cv2.DIST_C, 5)
        local_maxi = peak_local_max(distance, indices=False, footprint=np.ones((60, 60)), labels=self.image)
        markers = morphology.label(local_maxi)
        self.labels = watershed(-distance, markers, mask=self.image)
        # props = regionprops(labels_ws)
        i=0
        for label in np.unique(self.labels):
            if label == 0:
                continue

            mask = np.zeros(self.image.shape, dtype="uint8")
            mask[self.labels == label] = 255
            cnt, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            contours=[]
            corrected_conturs=[]
            for idx, val in enumerate(cnt):
                area = cv2.contourArea(val)
                if area>skip_area_less_than:
                    contours.append(val)
                    for idx1, val1 in enumerate(val):
                        for idx2, val2 in enumerate(val1):
                            corrected_conturs.append((cnt[idx][idx1][idx2][0],cnt[idx][idx1][idx2][1]))
            
            if len(contours)>0:
                self.conturs.append(Contur(contours,corrected_conturs, str(i),[label])) #self.labels[label]
                i+=1
        return self.conturs
