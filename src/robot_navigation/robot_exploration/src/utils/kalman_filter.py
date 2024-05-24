import time
import matplotlib

import math
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


import numpy as np
from numpy.linalg import inv 
import pandas as pd
import matplotlib.pyplot as plt


class KalmanFilter():
    def __init__(self, system_settings, initial_states = None):
        """
        The constructor for the kalman filter, it will take a tuple with the relevant system settings, 
        we will use the following notation.
        - R measurment noise covariance
        - Q system noise covariance
        - P state covariance
        - H observation matrix
        
        """
        (self.R, self.Q, self.P,
         self.H) = system_settings
        
        dims = self.P.shape[1]
        self.I = np.eye(dims, dtype= np.float32)
        self.x = np.zeros((dims, 1), dtype= np.float32) if initial_states is None else initial_states

    def predict(self, A = None,  B = None, u = 0, dt = 0.01):
        # Predict states based on systems dynamics
        x_dot = np.dot(A, self.x) + np.dot(B,u)
        self.x = self.x + x_dot*dt
        # Updated covariance by running covariance through system
        self.P = np.dot(np.dot(A,self.P),A.T) + self.Q
        return self.x

    def update(self, measurements):
        # Get error between measurements and output
        error = measurements - np.dot(self.H, self.x)
        # Update error covariance
        E = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        # Estimate Kalman Gain
        E_inv = np.linalg.inv(E)
        K = np.dot(np.dot(self.P, self.H.T), E_inv)
        # Update the state
        self.x = self.x + np.dot(K, error)
        # Update the state covariance
        self.P = np.dot(np.dot(self.I - np.dot(K, self.H), self.P), (self.I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)
        return self.x