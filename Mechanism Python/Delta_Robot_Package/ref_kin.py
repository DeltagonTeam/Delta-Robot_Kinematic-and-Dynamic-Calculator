'''
==========================================================================================================================
This module is for the reference Inverse Kinematics that are implemented using the sympy library
==========================================================================================================================
'''


# Importing:
'''Note: "from ... import *": imports all names from the file except for ones that start with underscore (conventionally private)'''
import sympy as sym
from Delta_Robot_Package.constants import *
from Delta_Robot_Package.end_effector_path import *
'''_______________________________________________________________________________________________________________________________________'''


# Reference Position Inverse Kinematics:
    # Helping Caclulations:
'''calculations that make the expressions of the motor angles more readable'''
ref_E1 = (2*L*(path_y+a))
ref_E2 = (-L*(sqrt3*(path_x+b)+path_y+c))
ref_E3 = (L*(sqrt3*(path_x-b)-path_y-c))

ref_F = (2*path_z*L)

ref_G1 = (path_x*path_x + path_y*path_y + path_z*path_z + a*a + L*L + 2*path_y*a - l*l)
ref_G2 = (path_x*path_x + path_y*path_y + path_z*path_z + b*b + c*c + L*L + 2*(path_x*b + path_y*c) - l*l)
ref_G3 = (path_x*path_x + path_y*path_y + path_z*path_z + b*b + c*c + L*L + 2*(-path_x*b + path_y*c) - l*l)

ref_under_sqrt1 = (ref_E1*ref_E1+ref_F*ref_F-ref_G1*ref_G1)
ref_under_sqrt2 = (ref_E2*ref_E2+ref_F*ref_F-ref_G2*ref_G2)
ref_under_sqrt3 = (ref_E3*ref_E3+ref_F*ref_F-ref_G3*ref_G3)

ref_tangent1a = ((-ref_F+sym.sqrt(ref_under_sqrt1))/(ref_G1-ref_E1))
ref_tangent2a = ((-ref_F+sym.sqrt(ref_under_sqrt2))/(ref_G2-ref_E2))
ref_tangent3a = ((-ref_F+sym.sqrt(ref_under_sqrt3))/(ref_G3-ref_E3))

ref_tangent1b = (-ref_F-sym.sqrt(ref_under_sqrt1))/(ref_G1-ref_E1)
ref_tangent2b = (-ref_F-sym.sqrt(ref_under_sqrt2))/(ref_G2-ref_E2)
ref_tangent3b = (-ref_F-sym.sqrt(ref_under_sqrt3))/(ref_G3-ref_E3)


    # Getting the Angles of the Motors:
ref_th_1a = (2*sym.atan(ref_tangent1a))
ref_th_2a = (2*sym.atan(ref_tangent2a))
ref_th_3a = (2*sym.atan(ref_tangent3a))

ref_th_1b = (2*sym.atan(ref_tangent1b))
ref_th_2b = (2*sym.atan(ref_tangent2b))
ref_th_3b = (2*sym.atan(ref_tangent3b))
'''_______________________________________________________________________________________________________________________________________'''


# Reference Velocity Inverse Kinematics:
'''calculating the angular velocities of the motors by diffrentiating the angular positions'''
ref_thd_1a = (sym.diff(ref_th_1a, t, 1))
ref_thd_2a = (sym.diff(ref_th_2a, t, 1))
ref_thd_3a = (sym.diff(ref_th_3a, t, 1))

ref_thd_1b = (sym.diff(ref_th_1b, t, 1))
ref_thd_2b = (sym.diff(ref_th_2b, t, 1))
ref_thd_3b = (sym.diff(ref_th_3b, t, 1))
'''_______________________________________________________________________________________________________________________________________'''


# Reference Accelaration Inverse Kinematics:
'''calculating the angular accelerations of the motors by diffrentiating the angular velocities'''
ref_thdd_1a = (sym.diff(ref_thd_1a, t, 1))
ref_thdd_2a = (sym.diff(ref_thd_2a, t, 1))
ref_thdd_3a = (sym.diff(ref_thd_3a, t, 1))

ref_thdd_1b = (sym.diff(ref_thd_1b, t, 1))
ref_thdd_2b = (sym.diff(ref_thd_2b, t, 1))
ref_thdd_3b = (sym.diff(ref_thd_3b, t, 1))
'''_______________________________________________________________________________________________________________________________________'''