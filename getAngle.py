import math
import numpy as np

class getAngle:
    def __init__(self):
        self.pointList = []
        self.pointCount = 0
        self.angle = 0
        self.pointSkip = 4
        self.counter = 0

        self.savedAngles = []
    
    def angle_reset(self):
        self.pointList = []
        self.pointCount = 0
        self.angle = 0
        self.counter = 0

        self.savedAngles = []        

    def get_angle(self, x, y):
        self.counter += 1
        if self.pointCount == 0:
            self.pointList.append((x,y))
            self.pointCount += 1
            if self.counter < self.pointSkip:
                self.angle = 0
            self.savedAngles.append(self.angle)
            return self.angle
        else:
            self.pointList.append((x,y))
            self.pointCount += 1
            if self.pointCount == self.pointSkip:
                dy = self.pointList[-1][1] - self.pointList[0][1]
                dx = self.pointList[-1][0] - self.pointList[0][0]
                #self.angle = -(math.degrees(math.atan2(dy,dx)))
                self.angle = -math.atan2(dy,dx)

                self.pointList = []
                self.pointCount = 0

                self.savedAngles.append(self.angle)
                return self.angle
            else:
                self.savedAngles.append(self.angle)
                return self.angle
