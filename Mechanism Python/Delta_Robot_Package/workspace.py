'''
==========================================================================================================================
The Module that Calculates the Workspace of the Delta Robot
==========================================================================================================================
'''


# Importing:
'''Note: "from ... import *": imports all names from the file except for ones that start with underscore (conventionally private)'''
import numpy as np      # for arrays and matrix operations
from Delta_Robot_Package.dir_kin import *
from Delta_Robot_Package.constants import *     # the dimentional constants of the delta robot
from Delta_Robot_Package.inv_kin import *       # the inverse kinematics of the delta robot
'''_______________________________________________________________________________________________________________________________________'''


# Defining Functions:
'''
* Function	    : get_work_volume   : calculates the work volume and creates a figure for it
* Input(s)      :           	: 
* Return(s)     :       	    : 
'''
def get_work_volume(n):
    X = np.full(n*n*n,None)
    Y = np.full(n*n*n,None)
    Z = np.full(n*n*n,None)

    counter = 0
    for TH1 in np.linspace(min_angle,max_angle,n):
        for TH2 in np.linspace(min_angle,max_angle,n):
            for TH3 in np.linspace(min_angle,max_angle,n):
                [x,y,z],error = get_coords([TH1,TH2,TH3],solution='lower')
                
                if error: continue

                X[counter],Y[counter],Z[counter] = x,y,z
                counter += 1

    return np.copy(X[0:counter]),np.copy(Y[0:counter]),np.copy(Z[0:counter])

'''
* Function	    : get_work_area     : calculates the work area at a certain z-level and creates a figure for it
* Input(s)      :           	: 
* Return(s)     :       	    : 
'''
def get_work_area(xy_range,z_layer,n):
    X = np.full(n*n,None)
    Y = np.full(n*n,None)

    counter = 0
    for x in np.linspace(-xy_range,xy_range,n):
        for y in np.linspace(-xy_range,xy_range,n):
            thetas, error = get_thetas([x,y,z_layer])
            th_1,th_2,th_3 = thetas

            if error: continue       
            if (max_angle < th_1 or min_angle > th_1) or (max_angle < th_2 or min_angle > th_2) or (max_angle < th_3 or min_angle > th_3): continue    

            X[counter],Y[counter] = x,y
            counter += 1
    
    return np.copy(X[0:counter]),np.copy(Y[0:counter])
'''_______________________________________________________________________________________________________________________________________'''
