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


class Conturs:
    def __init__(self, image):
        self.conturs = []
        self.image = image

    def show(self, img):
        for (c, corrected_contur) in self.conturs:
            cv2.drawContours(img, c, -1, (0, 0, 255), 1, 1)

    def get_conturs(self):
        distance = cv2.distanceTransform(self.image, cv2.DIST_C, 5)
        local_maxi = peak_local_max(distance, indices=False, footprint=np.ones((60, 60)), labels=self.image)
        markers = morphology.label(local_maxi)
        labels_ws = watershed(-distance, markers, mask=self.image)

        for label in np.unique(labels_ws):
            if label == 0:
                continue

            # draw label on the mask
            mask = np.zeros(self.image.shape, dtype="uint8")
            mask[labels_ws == label] = 255
            cnt, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            contours=[]
            corrected_conturs=[]
            for idx, val in enumerate(cnt):
                area = cv2.contourArea(val)
                if area>0:
                    contours.append(val)
                    for idx1, val1 in enumerate(val):
                        for idx2, val2 in enumerate(val1):
                            corrected_conturs.append((cnt[idx][idx1][idx2][0],cnt[idx][idx1][idx2][1]))

            self.conturs.append((contours,corrected_conturs))

        return self.conturs
