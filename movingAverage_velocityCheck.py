import numpy as np


class movingAverage_velocityCheck:
    def __init__(self):
        self.count = 0
        self.window_size =150
        self.window_x = []
        self.window_y = []

    def reset(self):
        self.window_x = []
        self.window_y = []

    def apply_filter(self, axis, valX, valY, dt, speed, angle):

        self.window_x.append(valX)
        self.window_y.append(valY)

        if len(self.window_x) == self.window_size:
            del self.window_x[0]
            del self.window_y[0]

        value_x = np.mean(self.window_x)
        value_y = np.mean(self.window_y)
        point = {}

        if speed < 2:
            point['x'] = value_x
            point['y'] = value_y
        # elif speed >= 2 and speed < 4:
        #     proportion = speed - 2
            
        #     point['x'] = (1 - proportion) * value_x + proportion * valX
        #     point['y'] = (1 - proportion) * value_y + proportion * valY           
        #     print(f"Actual x: {valX}, Filtered X: {value_x}, final x: {point['x']}, speed: {proportion}")
        #     print(f"Actual y: {valY}, Filtered Y: {value_y}, final y: {point['y']}")
        else:
            # print(value_x)
            # print(value_y)
            #point['y'] = valY
            # print(valX, valY)
            # print("="*50)

            point['x'] = valX
            point['y'] = valY

            self.window_x = []
            self.window_y = []


        return int(point[axis])