import numpy as np
 
# Author: Addison Sears-Collins
# https://automaticaddison.com
# Description: Extended Kalman Filter example (two-wheeled mobile robot)
 

class EKF:
    def __init__(self):
        # Supress scientific notation when printing NumPy arrays
        np.set_printoptions(precision=3,suppress=True)
        self.rawData = []
        self.filteredData = []
        self.speedData = []

        self.R_factor = 20
        self.Q_factor = .9
        self.W_factor = 0.0

        self.state_estimate_k_minus_1 = np.array([0.0,0.0,0.0])
        
        self.A_k_minus_1 = np.array([[1.0,  0,   0],
                                                        [  0,1.0,   0],
                                                        [  0,  0, 1.0]])
        

        self.process_noise_v_k_minus_1 = np.array([0.01,0.01,0.003])
            

        self.Q_k = np.array([[2,   0,   0],
                                        [  0, 2,   0],
                                        [  0,   0, 50]]) * self.Q_factor
                        

        self.H_k = np.array([[1.0,  0,   0],
                                        [  0,1.0,   0],
                                        [  0,  0, 1.0]])
                                

        self.R_k = np.array([[1600,   0,    0],
                                [  0, 1600,    0],
                                [  0,    0, 1]]) * self.R_factor 

        self.sensor_noise_w_k = np.array([10,10,0.04]) * self.W_factor 


        self.control_vector_k_minus_1 = np.array([1.0,0.0]) * 0
        

        self.P_k_minus_1 = np.array([[0.1,  0,   0],
                                                        [  0,0.1,   0],
                                                        [  0,  0, 0.1]])
    def reset(self):
        # Supress scientific notation when printing NumPy arrays
        np.set_printoptions(precision=3,suppress=True)
        self.rawData = []
        self.filteredData = []
        self.speedData = []



        self.state_estimate_k_minus_1 = np.array([0.0,0.0,0.0])
        
        self.A_k_minus_1 = np.array([[1.0,  0,   0],
                                                        [  0,1.0,   0],
                                                        [  0,  0, 1.0]])
        

        self.process_noise_v_k_minus_1 = np.array([0.01,0.01,0.003])
            

        self.Q_k = np.array([[2,   0,   0],
                                        [  0, 2,   0],
                                        [  0,   0, 50]]) * self.Q_factor
                        

        self.H_k = np.array([[1.0,  0,   0],
                                        [  0,1.0,   0],
                                        [  0,  0, 1.0]])
                                

        self.R_k = np.array([[1600,   0,    0],
                                [  0, 1600,    0],
                                [  0,    0, 1]]) * self.R_factor 

        self.sensor_noise_w_k = np.array([10,10,0.04]) * self.W_factor 


        self.control_vector_k_minus_1 = np.array([1.0,0.0]) * 0
        

        self.P_k_minus_1 = np.array([[0.1,  0,   0],
                                                        [  0,0.1,   0],
                                                        [  0,  0, 0.1]])
                     

    def getB(self, yaw, deltak):
        """
        Calculates and returns the B matrix
        3x2 matix -> number of states x number of control inputs
        The control inputs are the forward speed and the
        rotation rate around the z axis from the x-axis in the 
        counterclockwise direction.
        [v,yaw_rate]
        Expresses how the state of the system [x,y,yaw] changes
        from k-1 to k due to the control commands (i.e. control input).
        :param yaw: The yaw angle (rotation angle around the z axis) in rad 
        :param deltak: The change in time from time step k-1 to k in sec
        """
        B = np.array([  [np.cos(yaw)*deltak, 0],
                                        [np.sin(yaw)*deltak, 0],
                                        [0, deltak]])
        return B
    
    def predict(self, dk):
        ######################### Predict #############################
        # Predict the state estimate at time k based on the state 
        # estimate at time k-1 and the control input applied at time k-1.
        self.state_estimate_k = self.A_k_minus_1 @ (
                self.state_estimate_k_minus_1) + (
                self.getB(self.state_estimate_k_minus_1[2],dk)) @ (
                self.control_vector_k_minus_1) + (
                self.process_noise_v_k_minus_1)
                
        #print(f'State Estimate Before EKF={self.state_estimate_k}')
                
        # Predict the state covariance estimate based on the previous
        # covariance and some noise
        self.P_k = self.A_k_minus_1 @ self.P_k_minus_1 @ self.A_k_minus_1.T + (
                self.Q_k)
        
        return self.state_estimate_k, self.P_k

    def correct(self,Z, speed):
        z_k_observation_vector = Z
        ################### Update (Correct) ##########################
        # Calculate the difference between the actual sensor measurements
        # at time k minus what the measurement model predicted 
        # the sensor measurements would be for the current timestep k.
        measurement_residual_y_k = z_k_observation_vector - (
                (self.H_k @ self.state_estimate_k) + (
                self.sensor_noise_w_k))
    
        #print(f'Observation={z_k_observation_vector}')



        # Calculate the measurement residual covariance
        S_k = self.H_k @ self.P_k @ self.H_k.T + self.R_k
            
        # Calculate the near-optimal Kalman gain
        # We use pseudoinverse since some of the matrices might be
        # non-square or singular.
        K_k = self.P_k @ self.H_k.T @ np.linalg.pinv(S_k)
            
        # Calculate an updated state estimate for time k
        self.state_estimate_k = self.state_estimate_k + (K_k @ measurement_residual_y_k)
        
        # Update the state covariance estimate for time k
        self.P_k = self.P_k - (K_k @ self.H_k @ self.P_k)
        
        # Print the best (near-optimal) estimate of the current state of the robot
        #print(f'State Estimate After EKF={self.state_estimate_k}')

        self.state_estimate_k_minus_1 = self.state_estimate_k
        self.P_k_minus_1 = self.P_k
    
        # Return the updated state and covariance estimates
        return self.state_estimate_k, self.P_k
        

    # def ekf(self, z_k_observation_vector, state_estimate_k_minus_1, 
    #         control_vector_k_minus_1, P_k_minus_1, dk):
    #     """
    #     Extended Kalman Filter. Fuses noisy sensor measurement to 
    #     create an optimal estimate of the state of the robotic system.
            
    #     INPUT
    #         :param z_k_observation_vector The observation from the Odometry
    #             3x1 NumPy Array [x,y,yaw] in the global reference frame
    #             in [meters,meters,radians].
    #         :param state_estimate_k_minus_1 The state estimate at time k-1
    #             3x1 NumPy Array [x,y,yaw] in the global reference frame
    #             in [meters,meters,radians].
    #         :param control_vector_k_minus_1 The control vector applied at time k-1
    #             3x1 NumPy Array [v,v,yaw rate] in the global reference frame
    #             in [meters per second,meters per second,radians per second].
    #         :param P_k_minus_1 The state covariance matrix estimate at time k-1
    #             3x3 NumPy Array
    #         :param dk Time interval in seconds
                
    #     OUTPUT
    #         :return state_estimate_k near-optimal state estimate at time k  
    #             3x1 NumPy Array ---> [meters,meters,radians]
    #         :return P_k state covariance_estimate for time k
    #             3x3 NumPy Array                 
    #     """
    #     ######################### Predict #############################
    #     # Predict the state estimate at time k based on the state 
    #     # estimate at time k-1 and the control input applied at time k-1.
    #     state_estimate_k = self.A_k_minus_1 @ (
    #             state_estimate_k_minus_1) + (
    #             self.getB(state_estimate_k_minus_1[2],dk)) @ (
    #             control_vector_k_minus_1) + (
    #             self.process_noise_v_k_minus_1)
                
    #     print(f'State Estimate Before EKF={state_estimate_k}')
                
    #     # Predict the state covariance estimate based on the previous
    #     # covariance and some noise
    #     P_k = self.A_k_minus_1 @ P_k_minus_1 @ self.A_k_minus_1.T + (
    #             self.Q_k)
            
    #     ################### Update (Correct) ##########################
    #     # Calculate the difference between the actual sensor measurements
    #     # at time k minus what the measurement model predicted 
    #     # the sensor measurements would be for the current timestep k.
    #     measurement_residual_y_k = z_k_observation_vector - (
    #             (self.H_k @ state_estimate_k) + (
    #             self.sensor_noise_w_k))
    
    #     print(f'Observation={z_k_observation_vector}')



    #     # Calculate the measurement residual covariance
    #     S_k = self.H_k @ P_k @ self.H_k.T + self.R_k
            
    #     # Calculate the near-optimal Kalman gain
    #     # We use pseudoinverse since some of the matrices might be
    #     # non-square or singular.
    #     K_k = P_k @ self.H_k.T @ np.linalg.pinv(S_k)
            
    #     # Calculate an updated state estimate for time k
    #     state_estimate_k = state_estimate_k + (K_k @ measurement_residual_y_k)
        
    #     # Update the state covariance estimate for time k
    #     P_k = P_k - (K_k @ self.H_k @ P_k)
        
    #     # Print the best (near-optimal) estimate of the current state of the robot
    #     print(f'State Estimate After EKF={state_estimate_k}')
    
    #     # Return the updated state and covariance estimates
    #     return state_estimate_k, P_k
        
    def apply_filter(self, axis, valX, valY, dt, speed, angle):
    
        # We start at time k=1
        k = 1
        
        # Time interval in seconds
        dk = dt

        current_measurement = np.array([valX, valY, angle])

        current_prediction, current_p = self.predict(dk)

        value = {}

        value['x'], value['y'], value['angle'] = current_prediction[0], current_prediction[1], current_prediction[2]

        print(valX, valY, int(value[axis]), " | ", "angle=", angle, value['angle'])
        
        return int(value[axis])


        
                                
        # Start at k=1 and go through each of the 5 sensor observations, 
        # one at a time. 
        # We stop right after timestep k=5 (i.e. the last sensor observation)
        # for k, obs_vector_z_k in enumerate(z_k,start=1):
        
        #     # Print the current timestep
        #     print(f'Timestep k={k}')  

        #     # Run the Extended Kalman Filter and store the 
        #     # near-optimal state and covariance estimates
        #     optimal_state_estimate_k, covariance_estimate_k = self.ekf(
        #         obs_vector_z_k, # Most recent sensor measurement
        #         self.state_estimate_k_minus_1, # Our most recent estimate of the state
        #         self.control_vector_k_minus_1, # Our most recent control input
        #         self.P_k_minus_1, # Our most recent state covariance matrix
        #         dk) # Time interval
            
        #     # Get ready for the next timestep by updating the variable values
        #     self.state_estimate_k_minus_1 = optimal_state_estimate_k
        #     self.P_k_minus_1 = covariance_estimate_k

        #     input("done")
            
        #     # Print a blank line
        #     print()
    