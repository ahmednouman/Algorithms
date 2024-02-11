import numpy as np

class LowPassFilter_velocityCheck:
    def __init__(self):
        self.filtered = {}
        self.filtered['x'] = None
        self.filtered['y'] = None
        self.consec_low_count = 0
        self.consec_low_count_thresh = 300
        
    def reset(self):
        self.filtered = {}
        self.filtered['x'] = None
        self.filtered['y'] = None
        

    def apply_filter(self, axis, valX, valY, dt, speed, angle):
        #self.alpha = 3/400  #0.0075
        #print(speed)
        constant = 0.0165
        self.alpha = constant * speed 
        if speed < 2:
            self.alpha = constant
            
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
            return round(self.filtered[axis])
        self.filtered['x'] = previous_value_x
        self.filtered['y'] = previous_value_y

        # if speed < 2:
        #     self.consec_low_count += 1
        #     if self.consec_low_count >= self.consec_low_count_thresh:
        #         self.filtered['x'] = previous_value_x
        #         self.filtered['y'] = previous_value_y
        #     else:
        #         value = {}
        #         value['x'] = valX
        #         value['y'] = valY
        #         self.reset()  
        #         return int(value[axis])                
        # else:
        #     value = {}
        #     value['x'] = valX
        #     value['y'] = valY
        #     self.reset()  
        #     self.consec_low_count = 0 
        #     return int(value[axis])
        # print(valX, valY)          
        # print(self.filtered)
        return round(self.filtered[axis])