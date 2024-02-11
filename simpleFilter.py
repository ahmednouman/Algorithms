import numpy as np


class simpleFilter:
    def __init__(self):
        self.speedhist = []
        self.speedWindow = 50
        self.speedavg = []
        self.speedmed = []
        self.count = 0

    def reset(self):
        self.speedhist = []
        self.speedavg = []
        self.speedmed = []


    def apply_filter(self, axis, valX, valY, dt, speed, angle):
        self.count += 1
        self.speedhist.append(speed)
        if len(self.speedhist) == self.speedWindow:
            del self.speedhist[0]
        speedAVG = np.mean(self.speedhist)
        speedMED = np.median(self.speedhist)
        self.speedavg.append(speedAVG)
        self.speedmed.append(speedMED)

        point = {}
        point['x'] = valX
        point['y'] = valY

        # if speed < 2:
        #     point[axis] = point[axis] + 290

        return int(point[axis])