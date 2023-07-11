'''
==========================================================================================================================
The Module with the Kinematic Model of the Delta Robot
==========================================================================================================================
'''


# Importing:
'''Note: "from ... import *": imports all names from the file except for ones that start with underscore (conventionally private)'''
import sympy as sym
import numpy as np
from Delta_Robot_Package.constants import *
from Delta_Robot_Package.inv_kin import *
'''_______________________________________________________________________________________________________________________________________'''

# Defining Symbols:
X,Y,Z, TH1,TH2,TH3 = sym.symbols('x,y,z, th1,th2,th3', real=True)
'''_______________________________________________________________________________________________________________________________________'''


# Defining the Equations of the Kinematic Model of the Delta Robot:
f1 = sym.Eq(2*L*(Y+a)*sym.cos(TH1) + 2*Z*L*sym.sin(TH1)\
    + X*X + Y*Y + Z*Z + a*a + L*L + 2*Y*a - l*l, 0)
f2 = sym.Eq(-L*(sqrt3*(X+b)+Y+c)*sym.cos(TH2) + 2*Z*L*sym.sin(TH2)\
    + X*X + Y*Y + Z*Z + b*b + c*c + L*L + 2*X*b + 2*Y*c - l*l,0)
f3 = sym.Eq(L*(sqrt3*(X-b)-Y-c)*sym.cos(TH3) + 2*Z*L*sym.sin(TH3)\
    + X*X + Y*Y + Z*Z + b*b + c*c + L*L - 2*X*b + 2*Y*c - l*l,0)
'''_______________________________________________________________________________________________________________________________________'''


# Defining Functions:
'''
* Function  : get_thetas_numerically    : a public function that calculates the coordinates of the end effector numerically from the kinematic model
* Input1    : thetas                    : a list with the anglular positions of the motors
* Input2    : initial_guess             : the initial guess for the numerical solver
* Return1   : coords		            : a list with the coordinates of end effector
* Return2   : error	                    : whether there is an error
'''
def get_coords_numerically(thetas, initial_guess=[0,0,-l]):
    # Unpacking Inputs:
    th_1,th_2,th_3 = thetas

    # Getting the Equations to be Solved:
    '''substituting with the currect given coordinates in the equations of the model'''
    equation1 = f1.subs([(TH1,th_1)])
    equation2 = f2.subs([(TH2,th_2)])
    equation3 = f3.subs([(TH3,th_3)])

    # Solving the Equations:
    coords = sym.nsolve([equation1,equation2,equation3], (X,Y,Z), (initial_guess))

    # Checking if Any of the Betas Passed their Maximum Allowed Angle:
    betas = get_betas(coords, thetas)
    beta1,beta2,beta3 = betas
    if max_beta<beta1 or max_beta<beta2 or max_beta<beta3: return coords, True

    # Returning:
    return coords, False

'''
* Function  : get_thetas_numerically    : a public function that calculates the anglur positions of the motors numerically from the kinematic model
* Input1    : coords		            : a list with the coordinates of end effector
* Input2    : initial_guess             : the initial guess for the numerical solver
* Return1   : thetas                    : a list with the anglular positions of the motors
* Return2   : error	                    : whether there is an error
'''
def get_thetas_numerically(coords, initial_guess=[0,0,0]):
    # Unpacking Inputs:
    x,y,z = coords

    # Getting the Equations to be Solved:
    '''substituting with the currect given coordinates in the equations of the model'''
    equation1 = f1.subs([(X,x), (Y,y), (Z,z)])
    equation2 = f2.subs([(X,x), (Y,y), (Z,z)])
    equation3 = f3.subs([(X,x), (Y,y), (Z,z)])

    # Solving the Equations:
    thetas = sym.nsolve([equation1,equation2,equation3], (TH1,TH2,TH3), (initial_guess))

    # Checking if Any of the Betas Passed their Maximum Allowed Angle:
    beta1,beta2,beta3 = get_betas(coords, thetas)
    if max_beta<beta1 or max_beta<beta2 or max_beta<beta3: return thetas, True

    # Returning:
    return thetas, False
'''_______________________________________________________________________________________________________________________________________'''