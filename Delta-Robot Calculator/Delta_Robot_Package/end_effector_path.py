'''
==========================================================================================================================
The Module that Defines the Path of the End Effector
==========================================================================================================================
'''


# Importing:
import sympy as sym
'''_______________________________________________________________________________________________________________________________________'''


# Defining Symbols:
t = sym.symbols('t',real=True)  # define the t as a symbol that is written as 't'
'''_______________________________________________________________________________________________________________________________________'''


# Generating End Effector Path:
'''defining the path that the end effector will take to get the motor angles based on it'''
path_x = (t*t)/1000
path_xd = sym.diff(path_x)
path_xdd = sym.diff(path_xd)
path_y = (t*t)/1000
path_yd = sym.diff(path_y)
path_ydd = sym.diff(path_yd)
path_z = (t*t - 400)/1000
path_zd = sym.diff(path_z)
path_zdd = sym.diff(path_zd)
'''_______________________________________________________________________________________________________________________________________'''