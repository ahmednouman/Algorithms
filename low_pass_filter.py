import numpy as np

class LowPassFilter:
    def __init__(self):
        self.filtered = {}
        self.filtered['x'] = None
        self.filtered['y'] = None
        
    def reset(self):
        self.filtered = {}
        self.filtered['x'] = None
        self.filtered['y'] = None

    def apply_filter(self, axis, valX, valY, dt, speed, angle):
        self.alpha = 3/400
        previous_value_x = self.filtered["x"]
        previous_value_y = self.filtered["y"]
        if (self.filtered['x'] != None):        
            previous_value_x += (self.alpha * (valX - previous_value_x))
            previous_value_y += (self.alpha * (valY - previous_value_y))
        else:
            previous_value_x = valX
            previous_value_y = valY
            self.filtered['x'] = previous_value_x
            self.filtered['y'] = previous_value_y
            return int(self.filtered[axis])
        self.filtered['x'] = previous_value_x
        self.filtered['y'] = previous_value_y
        return int(self.filtered[axis])