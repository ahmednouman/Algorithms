import numpy as np

# def dynamic_moving_average(data):
#     window_avg = round(np.mean(data))   
#     return window_avg

class MovingAverageFilter:
    def __init__(self):
        self.window_size = 200
        self.history = {}
        self.history['x'] = []
        self.history['y'] = []

    def reset(self):
        self.history = {}
        self.history['x'] = []
        self.history['y'] = []        

    def apply_filter(self, axis, valX, valY, dt, speed, angle):
        self.history['x'].append(valX)
        self.history['y'].append(valY)
        if len(self.history[axis]) > self.window_size:
            del self.history['x'][0]
            del self.history['y'][0]

        value_x = np.mean(self.history['x'])
        value_y = np.mean(self.history['y'])      

        point = {}
        point['x'] = value_x
        point['y'] = value_y

        return int(point[axis])  