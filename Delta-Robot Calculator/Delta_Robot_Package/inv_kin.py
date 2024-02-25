'''
==========================================================================================================================
The Module that Calculates the Inverse Kinematics of the Delta Robot
==========================================================================================================================
'''


# Importing:
'''Note: "from ... import *": imports all names from the file except for ones that start with underscore (conventionally private)'''
import math
from Delta_Robot_Package.constants import *
from Delta_Robot_Package.internal_kin import *
'''_______________________________________________________________________________________________________________________________________'''


# Defining Functions:
'''
* Function	: __calc_th		: a private function that calculates the anglur position of any motor
* Inputs    : E,F,G    		: precalculated expressions that are different for each motor
* Return1   : th          	: the anglular position of the motor
* Return2   : error	        : whether there is an error
'''
def __calc_th(E,F,G):
    # Checking whether a Real Solution Exists:
    under_sqrt = E*E + F*F - G*G
    if 0 > under_sqrt: return None, True    
    
    # Choosing the Solution to be Returned:
    tangenta = (-F+math.sqrt(under_sqrt))/(G-E)
    tangentb = (-F-math.sqrt(under_sqrt))/(G-E)

    if abs(tangenta) < abs(tangentb): tangent = tangenta 
    else: tangent = tangentb     

    # Returning:
    return 2*math.atan(tangent), False

'''
* Function  : get_thetas    : a public function that calculates the anglur positions of the motors
* Input     : coords		: a list with the coordinates of end effector
* Return1   : thetas        : a list with the anglular positions of the motors
* Return2   : error	        : whether there is an error
'''
def get_thetas(coords):
    # Unpacking Inputs:
    x,y,z = coords
    
    # Calculating Expression used for the Three Motor Angles:
    F  = 2*z*L

    # Getting theta 1:
    E = 2*L*(y+a)
    G = x*x + y*y + z*z + a*a + L*L + 2*y*a - l*l
    th_1,error1 = __calc_th(E,F,G)

    # Getting theta 2:
    E = -L*(sqrt3*(x+b)+y+c)
    G = x*x + y*y + z*z + b*b + c*c + L*L + 2*(x*b + y*c) - l*l
    th_2,error2 = __calc_th(E,F,G)

    # Getting theta 3:
    E = L*(sqrt3*(x-b)-y-c)
    G = x*x + y*y + z*z + b*b + c*c + L*L + 2*(-x*b + y*c) - l*l
    th_3,error3 = __calc_th(E,F,G)

    # Packing Thetas:
    thetas = [th_1,th_2,th_3]

    # Checking whether there is an Error with Any of the Angles:
    if error1 or error2 or error3: return thetas, True    

    # Checking if Any of the Betas Passed their Maximum Allowed Angle:
    beta1,beta2,beta3 = get_betas(coords, thetas)
    if max_beta<beta1 or max_beta<beta2 or max_beta<beta3: return thetas, True   
    
    # Returning:
    return thetas, False

'''
* Function	: get_theta_dots    : a public function that calculates the anglur velosities of the motors
* Input1    : coords            : a list with the coordinates of end effector
* Input2    : velocities        : a list with the velocity components of the end effector
* Input3    : thetas            : a list with the anglular positions of the motors
* Return	: theta_dots 	    : a list with the anglular velocities of the motors
'''
def get_theta_dots(coords, velocities, thetas):
    # Unpacking Inputs:
    x,y,z = coords
    xd,yd,zd = velocities
    th_1,th_2,th_3 = thetas

    # Calculating the Angular Velocities of the Motors:
    thd_1 = (L*(yd*math.cos(th_1) + zd*math.sin(th_1)) + x*xd + (y+a)*yd + z*zd) / (L*((y+a)*math.sin(th_1) - z*math.cos(th_1)))
    thd_2 = (L*((sqrt3*xd+yd)*math.cos(th_2) - 2*zd*math.sin(th_2)) - 2*((x+b)*xd + (y+c)*yd + z*zd)) / (L*((sqrt3*(x+b)+y+c)*math.sin(th_2) + 2*z*math.cos(th_2)))
    thd_3 = (L*((sqrt3*xd-yd)*math.cos(th_3) + 2*zd*math.sin(th_3)) + 2*((x-b)*xd + (y+c)*yd + z*zd)) / (L*((sqrt3*(x-b)-y-c)*math.sin(th_3) - 2*z*math.cos(th_3)))
    
    # Returning:
    theta_dots = [thd_1,thd_2,thd_3]
    return theta_dots

'''
* Function	: get_theta_double_dots : a public function that calculates the anglur accelerations of the motors
* Input1    : coords                : a list with the coordinates of end effector
* Input2    : velocities            : a list with the velocity components of the end effector
* Input3    : accelerations         : a list with the acceleration components of the end effector
* Input4    : thetas                : a list with the anglular positions of the motors
* Input5	: theta_dots 	        : a list with the anglular velocities of the motors
* Return	: theta_double_dots 	: a list with the anglular accelerations of the motors
'''
def get_theta_double_dots(coords, velocities, accelerations, thetas, theta_dots):
    # Unpacking Inputs:
    x,y,z = coords
    xd,yd,zd = velocities
    xdd,ydd,zdd = accelerations
    th_1,th_2,th_3 = thetas
    thd_1,thd_2,thd_3 = theta_dots

    # Calculating the Angular Accelerations of the Motors:
    thdd_1 = (L*math.cos(th_1)*(ydd + (2*zd - (y+a)*thd_1)*thd_1)\
        + L*math.sin(th_1)*(zdd - (2*yd + z*thd_1)*thd_1)\
            + xd*xd + x*xdd + yd*yd + (y+a)*ydd + zd*zd + z*zdd)\
                /(L*((y+a)*math.sin(th_1) - z*math.cos(th_1)))
    
    thdd_2 = (L*math.cos(th_2)*(sqrt3*xdd + ydd - ((sqrt3*(x+b)+y+c)*thd_2 + 4*zd)*thd_2)\
        - 2*L*math.sin(th_2)*(zdd + ((sqrt3*xd+yd) - z*thd_2)*thd_2)\
            - 2*(xd*xd + (x+b)*xdd + yd*yd + (y+c)*ydd + zd*zd + z*zdd))\
                /(L*((sqrt3*(x+b)+y+c)*math.sin(th_2) + 2*z*math.cos(th_2)))
    
    thdd_3 = ((L*math.cos(th_3)*(sqrt3*xdd - ydd - ((sqrt3*(x-b)-y-c)*thd_3 - 4*zd)*thd_3)\
        + 2*L*math.sin(th_3)*(zdd - ((sqrt3*xd-yd) + z*thd_3)*thd_3)\
            + 2*(xd*xd + (x-b)*xdd + yd*yd + (y+c)*ydd + zd*zd + z*zdd))\
                /(L*((sqrt3*(x-b)-y-c)*math.sin(th_3) - 2*z*math.cos(th_3))))

    # Returning:
    theta_double_dots = [thdd_1,thdd_2,thdd_3]
    return theta_double_dots
'''_______________________________________________________________________________________________________________________________________'''