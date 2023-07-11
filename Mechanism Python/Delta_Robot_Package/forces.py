'''
==========================================================================================================================
The Module that Calculates the forces of Links and torques on the motors in the Delta Robot
==========================================================================================================================
'''


# Importing:
'''Note: "from ... import *": imports all names from the file except for ones that start with underscore (conventionally private)'''
import math     # built-in python math module
import numpy as np      # for arrays and matrix operations
from Delta_Robot_Package.constants import *     # the dimentional constants of the delta robot
from Delta_Robot_Package.inv_kin import *       # the inverse kinematics of the delta robot
from Delta_Robot_Package.dir_kin import *       # the direct kinematics of the delta robot
'''_______________________________________________________________________________________________________________________________________'''


# Defining Functions:
def platform_forces(coords, accelerations, thetas):
    # Unpacking the Position, Velocity, and Acceleration of the End Effector:
    x,y,z = coords
    xdd,ydd,zdd = accelerations

    # Unpacking the Angular Positions of the Motors:
    th_1,th_2,th_3 = thetas

    # Defining the Unit vector Components of the Driven Link (l):
    l1x_l = (x)/l
    l2x_l = (x - 0.5*sqrt3*L*math.cos(th_2) + b)/l
    l3x_l = (x + 0.5*sqrt3*L*math.cos(th_3) - b)/l
    l1y_l = (y + L*math.cos(th_1) + a)/l
    l2y_l = (y - L*0.5*math.cos(th_2) + c)/l
    l3y_l = (y - L*0.5*math.cos(th_3) + c)/l
    l1z_l = (z + L*math.sin(th_1))/l
    l2z_l = (z + L*math.sin(th_2))/l
    l3z_l = (z + L*math.sin(th_3))/l

    # Calculating Commonly Used Expressions:
    '''Note: EX = expression'''
    EX1 = SP*0.5 + delta*0.25
    EX2 = SP*0.5 - delta*0.25
    EX3 = wP - sqrt3*0.25*delta
    EX4 = wP + sqrt3*0.25*delta

    # Calculating the Elements of the Main Matrix: 
    '''Note: pm = platform matrix'''
    pm11 = pm12 = l1x_l; pm13 = pm14 = l2x_l; pm15 = pm16 = l3x_l
    pm21 = pm22 = l1y_l; pm23 = pm24 = l2y_l; pm25 = pm26 = l3y_l
    pm31 = pm32 = l1z_l; pm33 = pm34 = l2z_l; pm35 = pm36 = l3z_l

    pm41 = pm42 = -uP*l1z_l;            pm43 = EX3*l2z_l;   pm44 = EX4*l2z_l;   pm45 = EX4*l3z_l;   pm46 = EX3*l3z_l
    pm51 = delta_2*l1z_l;pm52 = -pm51;  pm53 = -EX1*l2z_l;  pm54 = -EX2*l2z_l;  pm55 = EX2*l3z_l;   pm56 = EX1*l3z_l

    pm61 = -delta_2*l1y_l + uP*l1x_l;   pm62 = delta_2*l1y_l + uP*l1x_l
    pm63 = EX1*l2y_l - EX3*l2x_l;       pm64 = EX2*l2y_l - EX4*l2x_l
    pm65 = -EX2*l3y_l - EX4*l3x_l;      pm66 = -EX1*l3y_l - EX3*l3x_l

    # Defining the Platform Coefficients Matrix:
    platform_matrix = np.array([
        [pm11, pm12, pm13, pm14, pm15, pm16],
        [pm21, pm22, pm23, pm24, pm25, pm26],
        [pm31, pm32, pm33, pm34, pm35, pm36],
        [pm41, pm42, pm43, pm44, pm45, pm46],
        [pm51, pm52, pm53, pm54, pm55, pm56],
        [pm61, pm62, pm63, pm64, pm65, pm66]
    ])

    # Defining the Platform Known Vector:
    platform_known_vector = np.array([
        -mP*xdd,
        -mP*ydd,
        mP*g - mP*zdd,
        -mP*h*ydd,
        mP*h*xdd,
        0
    ])
    
    # Calculating the Force Vector Manitudes:
    platform_unknown_vector = np.linalg.solve(platform_matrix,platform_known_vector)

    # Getting the Force Vectors:
    FP11 = platform_unknown_vector[0]*np.array([
        -l1x_l,
        -l1y_l,
        -l1z_l
    ])
    FP12 = platform_unknown_vector[1]*np.array([
        -l1x_l,
        -l1y_l,
        -l1z_l
    ])

    FP21 = platform_unknown_vector[2]*np.array([
        -l2x_l,
        -l2y_l,
        -l2z_l
    ])
    FP22 = platform_unknown_vector[3]*np.array([
        -l2x_l,
        -l2y_l,
        -l2z_l
    ])

    FP31 = platform_unknown_vector[4]*np.array([
        -l3x_l,
        -l3y_l,
        -l3z_l
    ])
    FP32 = platform_unknown_vector[5]*np.array([
        -l3x_l,
        -l3y_l,
        -l3z_l    
    ])
    
    # Returning:
    return FP11,FP12,FP21,FP22,FP31,FP32
'''_______________________________________________________________________________________________________________________________________'''


def driven_forces(FP11,FP12,FP21,FP22,FP31,FP32):
    FA11,FA12,FA21,FA22,FA31,FA32 = -FP11,-FP12,-FP21,-FP22,-FP31,-FP32
    FA11[2],FA12[2],FA21[2],FA22[2],FA31[2],FA32[2] = FA11[2]+ml*g, FA12[2]+ml*g, FA21[2]+ml*g, FA22[2]+ml*g, FA31[2]+ml*g, FA32[2]+ml*g
    return FA11,FA12,FA21,FA22,FA31,FA32
'''_______________________________________________________________________________________________________________________________________'''


def driver_forces(FA11,FA12,FA21,FA22,FA31,FA32, thetas,theta_dots,theta_double_dots):
    # Unpacking the Angular Positions, Velocities, and Accelerations of the Motors:
    th_1,th_2,th_3 = thetas
    thd_1,thd_2,thd_3 = theta_dots
    thdd_1,thdd_2,thdd_3 = theta_double_dots

    # Rotation Matrices:
    rot_matrix_B_B1 = np.array([
        [-1,    0,  0],
        [0,     -1, 0],
        [0,     0,  1]
    ])
    rot_matrix_B_B2 = np.array([
        [0.5,       -0.5*sqrt3, 0],
        [0.5*sqrt3, 0.5,        0],
        [0,         0,          1]
    ])
    rot_matrix_B_B3 = np.array([
        [0.5       ,    0.5*sqrt3,  0],
        [-0.5*sqrt3,    0.5      ,  0],
        [0         ,    0        ,  1]
    ])

    rot_matrix_L1_B1 = np.array([
        [1, 0,                  0               ],
        [0, math.cos(th_1),     math.sin(th_1)  ],
        [0, -math.sin(th_1),    math.cos(th_1)  ]
    ])
    rot_matrix_L2_B2 = np.array([
        [1, 0,                  0               ],
        [0, math.cos(th_2),     math.sin(th_2)  ],
        [0, -math.sin(th_2),    math.cos(th_2)  ]
    ])
    rot_matrix_L3_B3 = np.array([
        [1, 0,                  0               ],
        [0, math.cos(th_3),     math.sin(th_3)  ],
        [0, -math.sin(th_3),    math.cos(th_3)  ]
    ])

    B1_FA11, B1_FA12 = np.matmul(rot_matrix_B_B1,FA11), np.matmul(rot_matrix_B_B1,FA12)
    B2_FA21, B2_FA22 = np.matmul(rot_matrix_B_B2,FA21), np.matmul(rot_matrix_B_B2,FA22)
    B3_FA31, B3_FA32 = np.matmul(rot_matrix_B_B3,FA31), np.matmul(rot_matrix_B_B3,FA32)
    B1_RcgL = np.matmul(rot_matrix_L1_B1,Li_RcgL)
    B2_RcgL = np.matmul(rot_matrix_L2_B2,Li_RcgL)
    B3_RcgL = np.matmul(rot_matrix_L3_B3,Li_RcgL)

    # Calculating the Accelerations of the Centers of Mass of the Driver Link (L):
    B1_aL1 = np.array([
        0,
        thdd_1*B1_RcgL[2] - thd_1*thd_1*B1_RcgL[1],
        -thdd_1*B1_RcgL[1] - thd_1*thd_1*B1_RcgL[2]
    ])
    B2_aL2 = np.array([
        0,
        thdd_2*B2_RcgL[2] - thd_2*thd_2*B2_RcgL[1],
        -thdd_2*B2_RcgL[1] - thd_2*thd_2*B2_RcgL[2]
    ])
    B3_aL3 = np.array([
        0,
        thdd_3*B3_RcgL[2] - thd_3*thd_3*B3_RcgL[1],
        -thdd_3*B3_RcgL[1] - thd_3*thd_3*B3_RcgL[2]
    ])

    # Calculating Driver Link (L) Reactions:
    B1_FB1 = np.array([
        mL*B1_aL1[0]-B1_FA11[0]-B1_FA12[0],
        mL*B1_aL1[1]-B1_FA11[1]-B1_FA12[1],
        mL*B1_aL1[2]-B1_FA11[2]-B1_FA12[2]-mL*g
    ])
    B1_MB1 = np.array([
        -Li_ILx*thdd_1-L*math.cos(th_1)*B1_FA11[2]-L*math.sin(th_1)*B1_FA11[1]-L*math.cos(th_1)*B1_FA12[2]-L*math.sin(th_1)*B1_FA12[1]-mL*g*B1_RcgL[1],
        delta_2*B1_FA11[2]+L*math.sin(th_1)*B1_FA11[0]-delta_2*B1_FA12[2]+L*math.sin(th_1)*B1_FA12[0],
        -delta_2*B1_FA11[1]+L*math.cos(th_1)*B1_FA11[0]+delta_2*B1_FA12[1]+L*math.cos(th_1)*B1_FA12[0]
    ])

    B2_FB2 = np.array([
        mL*B2_aL2[0]-B2_FA21[0]-B2_FA22[0],
        mL*B2_aL2[1]-B2_FA21[1]-B2_FA22[1],
        mL*B2_aL2[2]-B2_FA21[2]-B2_FA22[2]-mL*g
    ])
    B2_MB2 = np.array([
        -Li_ILx*thdd_2-L*math.cos(th_2)*B2_FA21[2]-L*math.sin(th_2)*B2_FA21[1]-L*math.cos(th_2)*B2_FA22[2]-L*math.sin(th_2)*B2_FA22[1]-mL*g*B2_RcgL[1],
        delta_2*B2_FA21[2]+L*math.sin(th_2)*B2_FA21[0]-delta_2*B2_FA22[2]+L*math.sin(th_2)*B2_FA22[0],
        -delta_2*B2_FA21[1]+L*math.cos(th_2)*B2_FA21[0]+delta_2*B2_FA22[1]+L*math.cos(th_2)*B2_FA22[0]
    ])

    B3_FB3 = np.array([
        mL*B3_aL3[0]-B3_FA31[0]-B3_FA32[0],
        mL*B3_aL3[1]-B3_FA31[1]-B3_FA32[1],
        mL*B3_aL3[2]-B3_FA31[2]-B3_FA32[2]-mL*g
    ])
    B3_MB3 = np.array([
    -Li_ILx*thdd_3-L*math.cos(th_3)*B3_FA31[2]-L*math.sin(th_3)*B3_FA31[1]-L*math.cos(th_3)*B3_FA32[2]-L*math.sin(th_3)*B3_FA32[1]-mL*g*B3_RcgL[1],
    delta_2*B3_FA31[2]+L*math.sin(th_3)*B3_FA31[0]-delta_2*B3_FA32[2]+L*math.sin(th_3)*B3_FA32[0],
    -delta_2*B3_FA31[1]+L*math.cos(th_3)*B3_FA31[0]+delta_2*B3_FA32[1]+L*math.cos(th_3)*B3_FA32[0]
    ])

    # Returning:
    return B1_FB1,B1_MB1, B2_FB2,B2_MB2, B3_FB3,B3_MB3
'''_______________________________________________________________________________________________________________________________________'''

def bearing_reactions(B1_FB1,B1_MB1, B2_FB2,B2_MB2, B3_FB3,B3_MB3): # Distributing the Reactions on the Driver Link Revolute Joint as Bearing Reactions:
     # Defining the Bearing Reactions Known Vectors:
    bearing1_known_vector = np.array([
        B1_FB1[0],
        B1_FB1[1],
        B1_FB1[2],
        B1_MB1[0],
        B1_MB1[1],
        B1_MB1[2]
    ])
    bearing2_known_vector = np.array([
        B2_FB2[0],
        B2_FB2[1],
        B2_FB2[2],
        B2_MB2[0],
        B2_MB2[1],
        B2_MB2[2]
    ])
    bearing3_known_vector = np.array([
        B3_FB3[0],
        B3_FB3[1],
        B3_FB3[2],
        B3_MB3[0],
        B3_MB3[1],
        B3_MB3[2]
    ])


    # Defining the Bearing Reactions Coefficients Matrix:
    bearing_matrix = np.array([
        [1, 0,      0,      0,      0,      0],
        [0, 1,      1,      0,      0,      0],
        [0, 0,      0,      1,      1,      0],
        [0, 0,      0,      0,      0,      1],
        [0, 0,      0,      tau_2,  -tau_2, 0],
        [0, -tau_2, tau_2,  0,      0,      0]
    ]) 
    
    
    # Calculating the Bearing Reactions and Motor Torques:
    bearing1_unknown_vector = np.linalg.solve(bearing_matrix,bearing1_known_vector)
    bearing2_unknown_vector = np.linalg.solve(bearing_matrix,bearing2_known_vector)
    bearing3_unknown_vector = np.linalg.solve(bearing_matrix,bearing3_known_vector)

    return bearing1_unknown_vector, bearing2_unknown_vector, bearing3_unknown_vector

