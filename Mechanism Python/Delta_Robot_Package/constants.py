'''
==========================================================================================================================
The Module that has the Constants of the Delta Robot
==========================================================================================================================
'''


# Importing:
import math
import numpy as np
'''_______________________________________________________________________________________________________________________________________'''


# Defining Robot Dimentions:
SB = 360/1000    # base side length
SP = 240/1000    # platform side length
L  = 200/1000    # driver link length
l  = 400/1000    # driven link length
delta = 69/1000  # distance between the each pair of driven links
tau = 100/1000   # distance between the bearing of the driver link's revolute joint

max_beta = math.pi/6   # max angle for the spherical joints
min_angle = -math.pi/6  # the minimum angle allowed for the motors
max_angle = math.pi/3   # the maximum angle allowed for the motors
'''_______________________________________________________________________________________________________________________________________'''


# Defining Math Constants:
sqrt3 = math.sqrt(3)
'''_______________________________________________________________________________________________________________________________________'''


# Defining Helping Constants:
'''these are constants that symplify calculations for other modules'''
wB = (SB*sqrt3)/(6)
uB = (SB*sqrt3)/(3)
wP = (SP*sqrt3)/(6)
uP = (SP*sqrt3)/(3)

delta_2 = delta/2
tau_2 = tau/2

a = wB-uP
b = (SP-sqrt3*wB)/2
c = wP-0.5*wB
'''_______________________________________________________________________________________________________________________________________'''


# Defining Robot Inertial Information:
mP = 2000/1000     # platform mass
ml = (83+25*2)/1000      # driven link (l) mass
mL = 1500/1000      # driver link (L) mass
h = -50/1000     # platform center of mass height
g = -9.81    # gravitational acceleration
Li_ILx = 9143700/(10**9)  # driver link (L) moment of inertia around its axis of rotation
Li_RcgL = np.array([   # moment arm of center of mass of driver link (L) around its center of rotation with respect to its local coordinate system
    0/1000,
    87/1000,
    0/1000
])
'''_______________________________________________________________________________________________________________________________________'''