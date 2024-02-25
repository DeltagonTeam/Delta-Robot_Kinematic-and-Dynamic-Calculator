'''
==========================================================================================================================
The Module that Calculates the Direct Kinematics of the Delta Robot
==========================================================================================================================
'''


# Importing:
'''Note: "from ... import *": imports all names from the file except for ones that start with underscore (conventionally private)'''
import math
import numpy as np
import numpy.linalg as np_la
from Delta_Robot_Package.constants import *
from Delta_Robot_Package.internal_kin import *
'''_______________________________________________________________________________________________________________________________________'''


# Defining Functions:
'''
* Function	    : get_coords    : a public function that calculates the coordinates of the end effector
* Input1        : thetas        : a list with the anglular positions of the motors
* Input2        : solution      : a string that determines which solution is to be returned; valid inputs: 'lower', 'upper'
* Return1	    : coords        : a list with the coordinates of the end effector
* Return2       : error         : whether there is an error
'''
def get_coords(thetas, solution='lower'):
    # Unpacking Inputs:
    th_1,th_2,th_3 = thetas

    # Calculating the Center of Sphere 1 (Point A1):
    X1 = 0
    Y1 = -wB - L*math.cos(th_1) + uP
    Z1 = -L*math.sin(th_1)

    # Calculating the Center of Sphere 2 (Point A2):
    X2 = 0.5*(sqrt3*(wB + L*math.cos(th_2)) - SP)
    Y2 = 0.5*(wB + L*math.cos(th_2)) - wP
    Z2 = -L*math.sin(th_2)

    # Calculating the Center of Sphere 3 (Point A3):
    X3 = -0.5*(sqrt3*(wB + L*math.cos(th_3)) - SP)
    Y3 = 0.5*(wB + L*math.cos(th_3)) - wP
    Z3 = -L*math.sin(th_3)

    # Getting the End Effector Coordinates:
    A1,A2,A3 = [X1,Y1,Z1], [X2,Y2,Z2], [X3,Y3,Z3]
    radii = [l,l,l]
    coords,error = __trilaterate(A1,A2,A3, radii, solution)

    # Checking if Any of the Betas Passed their Maximum Allowed Angle:
    beta1,beta2,beta3 = get_betas(coords, thetas)
    if max_beta<beta1 or max_beta<beta2 or max_beta<beta3: error = True

    # Returning:
    return coords, error

'''
* Function	    : __trilaterate : a private function that calculates an intersection point of three spheres
* Input1        : A1            : a list with the coordinates of the center of sphere 1
* Input2        : A2            : a list with the coordinates of the center of sphere 2
* Input3        : A3            : a list with the coordinates of the center of sphere 3
* Input4        : radii         : a list with the radii of the three spheres
* Input5        : solution      : a string that determines which point is to be returned; valid inputs: 'lower', 'upper'
* Return1	    : coords        : a list with the coordinates of the choosen point of intersection
* Return2       : error         : whether there is an error
'''
def __trilaterate(A1,A2,A3, radii, solution):
    # Unpacking Inputs:
    [X1,Y1,Z1], [X2,Y2,Z2], [X3,Y3,Z3] = A1,A2,A3
    r1,r2,r3 = radii

    # Manipulating Equations:
    a11 = 2*(X3 - X1)
    a12 = 2*(Y3 - Y1)
    a13 = 2*(Z3 - Z1)

    a21 = 2*(X3 - X2)
    a22 = 2*(Y3 - Y2)
    a23 = 2*(Z3 - Z2)

    b1 = r1*r1 - r3*r3 - X1*X1 - Y1*Y1 - Z1*Z1 + X3*X3 + Y3*Y3 + Z3*Z3
    b2 = r2*r2 - r3*r3 - X2*X2 - Y2*Y2 - Z2*Z2 + X3*X3 + Y3*Y3 + Z3*Z3
    
    a1 = a22/a21 - a12/a11
    a2 = a13/a11 - a23/a21
    a3 = b2/a21 - b1/a11
    a4 = a2/a1
    a5 = a3/a1
    a6 = (a22*a4 + a23) / a21
    a7 = (b2 - a22*a5) / a21

    a0 = a6*a6 + a4*a4 + 1
    b0 = 2*(a6*(X1 - a7) + a4*(a5 - Y1) - Z1)
    c0 = a7*(a7 - 2*X1) + a5*(a5 - 2*Y1) + X1*X1 + Y1*Y1 + Z1*Z1 - r1*r1

    # Checking whether a Real Solution Exists:
    under_root = b0*b0 - 4*a0*c0
    if not under_root:
        return None,None,None, True

    # Choosing the Solution to be Returned:
    if 'upper' == solution:
        z = 0.5*(-b0 + math.sqrt(b0*b0 - 4*a0*c0)) / a0
    elif 'lower' == solution:
        z = 0.5*(-b0 - math.sqrt(b0*b0 - 4*a0*c0)) / a0
    else:
        raise Exception("the 'solution' parameter can only be 'upper' or 'lower'")
    
    # Calculating the Remaining Coordinates of the Solution:
    y = a4*z + a5
    x = -a6*z + a7
    coords = [x,y,z]

    # Returning:
    return coords, False
'''_______________________________________________________________________________________________________________________________________'''