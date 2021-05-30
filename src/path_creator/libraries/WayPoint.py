from marker_lib import *
from polygon_lib import *

class WayPoint:
    def __init__(self, contours, points, name):
        self.flannen_contours = contours
        self.points = points
        self.name = name
        self.markers = None
        self.polygon = None

        if len(contours)>0:
            self.polygon = Ploygon_lib('viz_' + name + '_poligons', contours)
        if len(self.points)>0:
            self.markers = Marker_lib('viz_' + name + '_markers', self.points)
        
    def display(self, contur=True, points=True):
        if contur and self.polygon != None:
            self.polygon.publish()

        if points and self.markers != None:
            self.markers.publish()


    