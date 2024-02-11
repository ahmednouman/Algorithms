import numpy as np
import math

class LowPassFilter_velocityCheck:
    def __init__(self):
        self.filtered = {}
        self.filtered['x'] = None
        self.filtered['y'] = None
        self.angle_history = []
        self.angle_avg_history = []
        self.consec_low_count = 0
        self.consec_low_count_thresh = 100
        
    def reset(self):
        self.filtered = {}
        self.filtered['x'] = None
        self.filtered['y'] = None
        self.angle_history = []
        self.angle_avg_history = []
        self.consec_low_count = 0
        
    def apply_filter(self, axis, valX, valY, dt, speed, angle):

        constant = .0185
        self.alpha = constant * speed 
        if speed < 1:
            self.alpha = constant
        else:
            self.alpha = 0.1
            
        previous_value_x = self.filtered["x"]
        previous_value_y = self.filtered["y"]

        if (self.filtered['x'] != None):        
            if speed < 1:
                self.consec_low_count += 1
                if self.consec_low_count >= self.consec_low_count_thresh:
                    self.alpha = constant * speed
                else:
                    if speed >= 1  and speed < 1.8:
                        self.alpha = constant * speed 
                    else:
                        self.alpha = 0.1
                        previous_value_x = valX
                        previous_value_y = valY
                        self.filtered['x'] = previous_value_x 
                        self.filtered['y'] = previous_value_y
                        return round(self.filtered[axis])

                # if speed <= 0.15:
                #     self.alpha = 0.1

                previous_value_x += (self.alpha * (valX - previous_value_x))
                previous_value_y += (self.alpha * (valY - previous_value_y))
                self.filtered['x'] = previous_value_x 
                self.filtered['y'] = previous_value_y
                return round(self.filtered[axis])
                        
            else:
                self.consec_low_count = 0
                previous_value_x += (self.alpha * (valX - previous_value_x))
                previous_value_y += (self.alpha * (valY - previous_value_y))
                self.filtered['x'] = previous_value_x 
                self.filtered['y'] = previous_value_y
                return round(self.filtered[axis])


        else:
            previous_value_x = valX
            previous_value_y = valY
            self.filtered['x'] = previous_value_x 
            self.filtered['y'] = previous_value_y

            return round(self.filtered[axis])
        self.filtered['x'] = previous_value_x
        self.filtered['y'] = previous_value_y

        return round(self.filtered[axis])