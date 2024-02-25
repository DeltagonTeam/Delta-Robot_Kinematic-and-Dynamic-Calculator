'''
==========================================================================================================================
This module calculates all the forces of the delta robot
==========================================================================================================================
'''


# Importing:
'''Note: "from ... import *": imports all names from the file except for ones that start with underscore (conventionally private)'''
import math     # built-in python math module
import numpy as np      # for arrays and matrix operations
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Delta_Robot_Package.constants import *     # the dimentional constants of the delta robot
from Delta_Robot_Package.inv_kin import *       # the inverse kinematics of the delta robot
from Delta_Robot_Package.dir_kin import *       # the direct kinematics of the delta robot
from Delta_Robot_Package.workspace import *
from Delta_Robot_Package.forces import *        # the force and torque calculations of the delta robot
'''_______________________________________________________________________________________________________________________________________'''

# Configuration:
    # Number of positions:
n_3d = 15
marker_size = 18


# Defining Functions:
def motor_forces(coords,velocities,accelerations):
    # Getting the Angular Positions, Velocities, and Accelerations of the Motors:
    thetas, error_thetas = get_thetas(coords)
    theta_dots = get_theta_dots(coords,velocities, thetas)
    theta_double_dots = get_theta_double_dots(coords,velocities,accelerations, thetas,theta_dots)
    
    FP11,FP12,FP21,FP22,FP31,FP32 = platform_forces(coords, accelerations, thetas)
    FA11,FA12,FA21,FA22,FA31,FA32 = driven_forces(FP11,FP12,FP21,FP22,FP31,FP32)
    B1_FB1,B1_MB1, B2_FB2,B2_MB2, B3_FB3,B3_MB3 = driver_forces(FA11,FA12,FA21,FA22,FA31,FA32, thetas,theta_dots,theta_double_dots)
    return bearing_reactions(B1_FB1,B1_MB1, B2_FB2,B2_MB2, B3_FB3,B3_MB3)


# The Main Function:
def main():
    # Defining the Position, Velocity, and Acceleration of the End Effector:
    X_3d,Y_3d,Z_3d = get_work_volume(n_3d)
    xd = 0
    xdd = 0
    yd = 0
    ydd = 0
    zd = 150/1000*0
    zdd = 9.81*0
    velocities = xd,yd,zd
    accelerations = xdd,ydd,zdd

    motor1_torque = np.full(len(X_3d),None)
    motor2_torque = np.full(len(X_3d),None)
    motor3_torque = np.full(len(X_3d),None)
    
    for i in np.arange(start=0,stop=len(X_3d),step=1):
        coords = X_3d[i],Y_3d[i],Z_3d[i]
        bearing1_unknown_vector,bearing2_unknown_vector,bearing3_unknown_vector = motor_forces(coords,velocities,accelerations)
        motor1_torque[i],motor2_torque[i],motor3_torque[i] = bearing1_unknown_vector[5],bearing2_unknown_vector[5],bearing3_unknown_vector[5]
    
    torques = np.array([motor1_torque,motor2_torque,motor3_torque])

    print(f'Torque 1 Max = {np.max(motor1_torque)} Nm')
    print(f'Torque 2 Max = {np.max(motor2_torque)} Nm')
    print(f'Torque 3 Max = {np.max(motor3_torque)} Nm')

    for j in np.arange(start=0,stop=3,step=1):
        value_array = np.abs(torques[j])
        cmap = mpl.cm.plasma
        norm = mpl.colors.Normalize(vmin=np.min(value_array),vmax=np.max(value_array))
        figure_torque = plt.figure(f'Motor {j+1} Torque')
        axes_torque = plt.axes(projection='3d')
        figure_torque.colorbar(mpl.cm.ScalarMappable(norm,cmap),ax=axes_torque,orientation='vertical',label='torque (Nm)')
        axes_torque.scatter(X_3d,Y_3d,Z_3d,c=value_array,cmap=cmap,s=marker_size)
        axes_torque.set_aspect('equal',adjustable='box')
        axes_torque.set_xlabel('X')
        axes_torque.set_ylabel('Y')
        axes_torque.set_zlabel('Z')
        axes_torque.view_init(elev=19, azim=34)
    plt.show()
    
    # Printing Results:
    '''
    print(f'Theta 1 = {thetas[0]*180/math.pi} deg')
    print(f'Theta 2 = {thetas[1]*180/math.pi} deg')
    print(f'Theta 3 = {thetas[2]*180/math.pi} deg')
    print(f'\n')
    print(f'Coordinates = {coords} m')
    print(f'\n')
    print(f'Torque 1 = {bearing1_unknown_vector[5]} Nm')
    print(f'Torque 2 = {bearing2_unknown_vector[5]} Nm')
    print(f'Torque 3 = {bearing3_unknown_vector[5]} Nm')
    print(f'\n')
    print(f'FP11 = {FP11} N')
    print(f'FP12 = {FP12} N')
    print(f'FP21 = {FP21} N')
    print(f'FP22 = {FP22} N')
    print(f'FP31 = {FP31} N')
    print(f'FP32 = {FP32} N')
    print(f'Total FP z = {FP11[2]+FP12[2] + FP21[2]+FP22[2] + FP31[2]+FP32[2]} N')
    print(f'\n')
    print(f'FA11 = {FA11} N')
    print(f'FA12 = {FA12} N')
    print(f'FA21 = {FA21} N')
    print(f'FA22 = {FA22} N')
    print(f'FA31 = {FA31} N')
    print(f'FA32 = {FA32} N')
    print(f'Total FA z = {FA11[2]+FA12[2] + FA21[2]+FA22[2] + FA31[2]+FA32[2]} N')
    print(f'\n')
    print(f'Bearing Reaction 1 = {B1_FB1[2]} N')
    print(f'Bearing Reaction 2 = {B2_FB2[2]} N')
    print(f'Bearing Reaction 3 = {B3_FB3[2]} N')
    print(f'Total Bearing Reaction z {B1_FB1[2]+B2_FB2[2]+B3_FB3[2]} N')
    print(f'\n')
    print(f'Bearing Reaction 11 = {bearing1_unknown_vector[3]} N')
    print(f'Bearing Reaction 12 = {bearing1_unknown_vector[4]} N')
    print(f'Bearing Reaction 21 = {bearing2_unknown_vector[3]} N')
    print(f'Bearing Reaction 22 = {bearing2_unknown_vector[4]} N')
    print(f'Bearing Reaction 31 = {bearing3_unknown_vector[3]} N')
    print(f'Bearing Reaction 32 = {bearing3_unknown_vector[4]} N')
    print(f'Total Bearing Reaction z {bearing1_unknown_vector[3]+bearing1_unknown_vector[4] + bearing2_unknown_vector[3]+bearing2_unknown_vector[4] + bearing3_unknown_vector[3]+bearing3_unknown_vector[4]} N')
    print(f'\n')
    print(f'Platform Weight = {9.81*mP} N')
    print(f'Driver Weight = {9.81*mL} N *3 = {9.81*mL*3} N')
    print(f'Driven Weight = {9.81*ml} N *6 = {9.81*ml*6} N')
    print(f'Total Weight = {9.81*(mP + mL*3 + ml*6)} N')
    print(f'\n')
    print(f'Calculated Torque = {9.81*((mP/3+ml*2)*L + mL*Li_RcgL[1])} Nm')
    '''

# Running the Main Function:
'''Note: "if __name__ == '__main__':" is a method used to ensure that this code does not run if imported to another file'''
if __name__ == '__main__':
    main()