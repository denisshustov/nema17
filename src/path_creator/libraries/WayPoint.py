from marker_lib import *
from polygon_lib import *

class WayPoint:
    def __init__(self, contours, points, name):
        self.flannen_contours = contours
        self.points = points
        self.name = name
        self.polygon = Ploygon_lib('viz_' + name + '_poligons', contours)
        self.markers = Marker_lib('viz_' + name + '_markers', self.points)
        
    def display(self, contur=True, points=True):
        if contur:
            self.polygon.publish()
        if points:
            self.markers.publish()


    