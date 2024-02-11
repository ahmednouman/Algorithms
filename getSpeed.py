import math
import numpy as np

class getSpeed:
    def __init__(self):
        self.speedhist = []
        self.speedWindow = 30
        self.speedavg = []
        self.speedmed = []

    def speed_reset(self):
        self.speedhist = []
        self.speedavg = []
        self.speedmed = []
      

    def get_speed(self, axis, speed, option=None):
        self.speedhist.append(speed)
        if len(self.speedhist) == self.speedWindow:
            del self.speedhist[0]
        speedAVG = np.mean(self.speedhist)
        speedMED = np.median(self.speedhist)
        self.speedavg.append(speedAVG)
        self.speedmed.append(speedMED)

        if option == "average":
            return speedAVG
        elif option == "median":
            return speedMED
        else:
            return speed