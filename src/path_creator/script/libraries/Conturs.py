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
    def __init__(self, contur, corrected_contur, id, label):
        self.contur = contur
        self.corrected_contur = corrected_contur
        self.id = id
        self.label = label

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

    def get_intersections(self, delta = 2):
        result = []
        for cnt in self.conturs:
            for cnt1 in self.conturs:
                if cnt.id != cnt1.id:
                    q = self.intersect2D(np.array(cnt.corrected_contur), \
                        np.array(cnt1.corrected_contur), delta)
                    if len(q)>0:
                        result.append((cnt.id,cnt1.id))
        return result

    def show(self, img):
        i=0
        for cnt in self.conturs:
            cv2.drawContours(img, cnt.contur, -1, (0, 0, 255), 1, 1)
            props = regionprops(self.labels)[i]
            x = int(props.centroid[1])
            y = int(props.centroid[0])
            cv2.putText(img, cnt.id, (x,y), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, \
                color=(255,0,255),thickness=2)
            i+=1

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

            # draw label on the mask
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

            self.conturs.append(Contur(contours,corrected_conturs, str(i),self.labels[label]))
            i+=1
        return self.conturs
