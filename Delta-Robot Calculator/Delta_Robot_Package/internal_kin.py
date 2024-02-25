'''
==========================================================================================================================
The Module that Calculates the Kinematics of Links and Vectors in the Delta Robot
==========================================================================================================================
'''


# Importing:
'''Note: "from ... import *": imports all names from the file except for ones that start with underscore (conventionally private)'''
import math
import numpy as np
import numpy.linalg as np_la
from Delta_Robot_Package.constants import *
'''_______________________________________________________________________________________________________________________________________'''


# Defining Functions:
'''
* Function	    : get_betas	: a public function that calculates the angles of deviation from the direction of limitless rotation for the sperical joints
* Input1        : coords    : a list with the coordinates of the end effector
* Input2        : thetas    : a list with the anglular positions of the motors
* Return	    : betas     : a list with the beta angles of the spherical joints
'''
def get_betas(coords, thetas):
    # Unpacking Inputs:
    x,y,z = coords
    th_1,th_2,th_3 = thetas

    # Getting the Unit Vectors in the Direction of Limitless Rotation:
    unitVect_u1 = np.array([1,    0,          0])
    unitVect_u2 = np.array([-0.5, 0.5*sqrt3,  0])
    unitVect_u3 = np.array([-0.5, -0.5*sqrt3, 0])

    # Getting the Vectors of the Driven Links (l):
    '''Note: if the '.astype(float)' is removed, an exception will be raised sometimes'''
    vect_l1 = np.array([x,                                   y + L*math.cos(th_1) + a,      z + L*math.sin(th_1)]).astype(float)
    vect_l2 = np.array([x - 0.5*sqrt3*L*math.cos(th_2) + b,  y - 0.5*L*math.cos(th_2) + c,  z + L*math.sin(th_2)]).astype(float)
    vect_l3 = np.array([x + 0.5*sqrt3*L*math.cos(th_3) - b,  y - 0.5*L*math.cos(th_3) + c,  z + L*math.sin(th_3)]).astype(float)

    # Getting the Unit Vectors of the Driven Links (l):
    unitVect_l1 = vect_l1 / np_la.norm(vect_l1)
    unitVect_l2 = vect_l2 / np_la.norm(vect_l2)
    unitVect_l3 = vect_l3 / np_la.norm(vect_l3)

    # Getting the Angles of Deviation from the Directions of Limitless Rotation:
    '''
    This is done by: 
    1 - Getting the dot product, which is just cos(angle) because both vectors are unit vectors.
    Note: the angle here is pi/2 - beta.
    2 - Taking the absolute value of that, since the direction of deviation does not matter here.
    3 - Getting the angle.
    4 - beta = pi/2 - angle
    '''
    beta1 = 0.5*math.pi - math.acos(abs(np.dot(unitVect_u1, unitVect_l1)))
    beta2 = 0.5*math.pi - math.acos(abs(np.dot(unitVect_u2, unitVect_l2)))
    beta3 = 0.5*math.pi - math.acos(abs(np.dot(unitVect_u3, unitVect_l3)))

    # Returning:
    betas = [beta1,beta2,beta3]
    return betas
'''_______________________________________________________________________________________________________________________________________'''