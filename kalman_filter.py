

import numpy as np
from numpy.linalg import inv


class KalmanFilter:
    """
    Simple Kalman filter
    """

    def __init__(self, B=np.array([0]), M=np.array([0])):
        """
        Initialise the filter
        Args:
            X: State estimate
            P: Estimate covariance
            F: State transition model
            B: Control matrix
            M: Control vector
            Q: Process noise covariance
            Z: Measurement of the state X
            H: Observation model
            R: Observation noise covariance
        """
        self.rawData = []
        self.filteredData = []
        self.speedData = []

        self.R_factor = 18000000
        self.Q_factor = 0.01

        self.X = np.array([[0], [0], [0], [0]], np.float32)
        self.P = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], np.float32) * 1
        self.F = np.array([[1, 0, 1, 0],[0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.B = B
        self.M = M
        self.Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], np.float32) * self.Q_factor 
        self.Z = np.zeros((2, 1), np.float32)
        self.H = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
        self.R = np.array([[1,0],[0,1]], np.float32) * self.R_factor
    
    def reset(self):
        self.X = np.array([[0], [0], [0], [0]], np.float32)
        self.P = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], np.float32) * 1
        self.F = np.array([[1, 0, 1, 0],[0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], np.float32) * self.Q_factor 
        self.Z = np.zeros((2, 1), np.float32)
        self.H = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
        self.R = np.array([[1,0],[0,1]], np.float32) * self.R_factor

    def predict(self, speed):
        """
        Predict the future state
        Args:
            self.X: State estimate
            self.P: Estimate covariance
            self.B: Control matrix
            self.M: Control vector
        Returns:
            updated self.X
        """
        # Project the state ahead
        # if speed >= 1:
        #     self.R_factor = 0
        # else:
        #     self.R = np.array([[1,0],[0,1]], np.float32) * self.R_factor
        #     self.Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], np.float32) * 200
        self.X = self.F @ self.X + self.B @ self.M
        self.P = self.F @ self.P @ self.F.T + self.Q

        return self.X

    def correct(self, Z, speed):
        """
        Update the Kalman Filter from a measurement
        Args:
            self.X: State estimate
            self.P: Estimate covariance
            Z: State measurement
        Returns:
            updated X
        """
        # if speed >= 4:
        #     self.R = np.array([[1,0],[0,1]], np.float32) * 000000
        # else:
        #     self.R = np.array([[1,0],[0,1]], np.float32) * self.R_factor
        K = self.P @ self.H.T @ inv(self.H @ self.P @ self.H.T + self.R)
        self.X += K @ (Z - self.H @ self.X)
        self.P = self.P - K @ self.H @ self.P

        return self.X

    def apply_filter(self, axis, valX, valY, dt, speed, angle):
        #print(valX, valY)
        current_measurement = np.array([[np.float32(valX)], [np.float32(valY)]])
        current_prediction = self.predict(speed)
        value = {}
        value['x'], value['y'] = current_prediction[0][0], current_prediction[1][0]
        return int(value[axis])
