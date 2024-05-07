import time
import matplotlib

import math
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


class RoverKF():
    def __init__(self):
        # Constants
        self.r = 0.04 # Wheel radius
        self.h = 0.15 # distance to turning point from wheels
        self.d = 0.10 # wheelbase

        self.sim_step = 0.0001

        self.theta = 0 # Initial angle for the rover
        self.states = np.mat([[0,0, self.theta]]).T #Initial points for the rover
        self.phi_mat = np.mat([[self.r/self.d, -self.r/self.d]])       

        self.inputs =  np.array([[0.1, 0.1]]).T

    def sim(self):
        out = self.states.T
        for i in np.arange(0.001, 10, self.sim_step):
            D_mat = self.get_angle_matrix()
            D_mat_inverse = (1/(D_mat[0,0]*D_mat[1,1]-D_mat[1,0]*D_mat[0,1]))*np.mat([[D_mat[1,1],-D_mat[0,1]],
                                                                                    [-D_mat[1,0],D_mat[0,0]]])
            
            M = np.array([ D_mat[0],
                           D_mat[1],
                           self.phi_mat[0]])
            
            breakpoint() 
            self.states = self.states + self.sim_step * (M@self.inputs)
            out = np.concatenate((out, self.states.T), axis = 0)

        out_df = pd.DataFrame(out, columns=["x","y","theta"])
        out_df.plot(x="x", y="y")


    
    def predict(self):
        D_mat = self.get_angle_matrix()
        D_mat_inverse = (1/(D_mat[0,0]*D_mat[1,1]-D_mat[1,0]*D_mat[0,1]))*np.mat([[D_mat[1,1],-D_mat[0,1]],
                                                                                    [-D_mat[1,0],D_mat[0,0]]])
        
    
    def update_meassurment(self):
        raise NotImplemented


            
    def get_angle_matrix(self):
        #print("Theta ", self.theta)
        return np.mat([[(self.r/2)*np.cos(self.theta)-((self.h*self.r)/self.d)*np.sin(self.theta), (self.r/2)*np.cos(self.theta)+((self.h*self.r)/self.d)*np.sin(self.theta)],
                       [(self.r/2)*np.sin(self.theta)+((self.h*self.r)/self.d)*np.cos(self.theta), (self.r/2)*np.sin(self.theta)-((self.h*self.r)/self.d)*np.cos(self.theta)]])
        
        
if __name__ == "__main__":
    rover = RoverKF()
    rover.sim()