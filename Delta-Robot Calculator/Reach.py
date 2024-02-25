'''
==========================================================================================================================
This caclulates the worksapce of the Delta-Robot
==========================================================================================================================
'''


# Importing:
'''Note: "from ... import *": imports all names from the file except for ones that start with underscore (conventionally private)'''
import matplotlib.pyplot as plt     # for plotting
import numpy as np      # for arrays and matrix operations
from Delta_Robot_Package.dir_kin import *
from Delta_Robot_Package.constants import *     # the dimentional constants of the delta robot
from Delta_Robot_Package.inv_kin import *       # the inverse kinematics of the delta robot
from Delta_Robot_Package.workspace import *
'''_______________________________________________________________________________________________________________________________________'''


# Configuration:
    # Modes to Run:
mode_3D = True
mode_2D = False

    # Number of positions:
n_3d = 15
n_2d = 120

    # 3D View Initial Angles:
view_elevation_angle = 90
view_azimuth_angle = 90

    # Marker Sizes in Figures:
marker_size_3d = 10
marker_size_2d = 5

    # 2D-Layer Mode Caclulation Configs:
xy_range = 300/1000      # how far to test for reachability in each direction
z_layer  = -450/1000     # the z level at which the section of the workspace is taken
'''_______________________________________________________________________________________________________________________________________'''


# The Main Function:
def main():

    if (True == mode_3D):

        X_3d,Y_3d,Z_3d = get_work_volume(n_3d)

        plt.figure('Delta_WorkVolume')
        axes_3d = plt.axes(projection='3d')
        axes_3d.scatter(X_3d,Y_3d,Z_3d,color='Darkcyan',s=marker_size_3d)
        axes_3d.set_aspect('equal',adjustable='box')
        axes_3d.set_xlabel('X')
        axes_3d.set_ylabel('Y')
        axes_3d.set_zlabel('Z')
        axes_3d.view_init(elev=view_elevation_angle, azim=view_azimuth_angle)
    
    if (True == mode_2D):

        X_2d,Y_2d = get_work_area(xy_range,z_layer,n_2d)

        plt.figure(f'Delta_WorkArea at Z = {z_layer}')
        axes_2d = plt.scatter(X_2d,Y_2d,color='Darkcyan',s=marker_size_2d).axes
        axes_2d.set_aspect(aspect='equal')
        axes_2d.set_xlabel('X')
        axes_2d.set_ylabel('Y')
      
    plt.show()
'''_______________________________________________________________________________________________________________________________________'''


# Running the Main Function:
'''Note: "if __name__ == '__main__':" is a method used to ensure that this code does not run if imported to another file'''
if __name__ == '__main__':
    main()