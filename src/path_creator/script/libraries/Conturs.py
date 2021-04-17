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

    def intersect2D(self, a, b, delta):
        result = []
        for a1 in a:
            for b1 in b:
                if abs(a1[0]-b1[0])<=delta and abs(a1[1]-b1[1])<=delta:
                    result.append((a1,b1))
        return result

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

    def get_intersections(self, delta = 2):
        for cnt in self.conturs:
            for cnt1 in self.conturs:
                if cnt.id != cnt1.id:
                    inters = self.intersect2D(np.array(cnt.corrected_contur), \
                        np.array(cnt1.corrected_contur), delta)
                    if len(inters)>0:
                        cnt.children.append(cnt1)
                        cnt1.parent = cnt
        return self.conturs

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
                cv2.putText(img, current_contur.id, (x,y), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, \
                    color=(255,0,255),thickness=2)

            current_conturs = self.get_contur_in_order(current_contur)
            if current_conturs == None:
                break
            current_contur = current_conturs[0]
        self.set_unvisited()


    def get_contur_in_order(self, current, go_from = None):
        if len(current.children)>0:
            for c in current.children:
                if not c.is_processed:
                    return [c]

            #all children processed
            for c in current.children:
                if go_from != None:
                    if go_from.id != c.id:
                        res = self.get_contur_in_order(c, current)
                        if res != None:
                            res.append(c)
                            res.append(current)
                            return res
                else:
                    res = self.get_contur_in_order(c, current)
                    if res != None:
                        res.append(c)
                        res.append(current)
                        return res

        return None

    def get_conturs(self):
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
                if area>10:
                    contours.append(val)
                    for idx1, val1 in enumerate(val):
                        for idx2, val2 in enumerate(val1):
                            corrected_conturs.append((cnt[idx][idx1][idx2][0],cnt[idx][idx1][idx2][1]))

            self.conturs.append(Contur(contours,corrected_conturs, str(i),[label])) #self.labels[label]
            i+=1
        return self.conturs
