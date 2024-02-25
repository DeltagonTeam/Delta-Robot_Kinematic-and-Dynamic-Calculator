'''
==========================================================================================================================
This tests the direct kinematics of the Delta-Robot
==========================================================================================================================
'''


# Importing:
'''Note: "from ... import *": imports all names from the file except for ones that start with underscore (conventionally private)'''
import matplotlib.pyplot as plt     # for plotting
import numpy as np      # for arrays and matrix operations
from Delta_Robot_Package.constants import *     # the dimentional constants of the delta robot
from Delta_Robot_Package.kin_model import *
from Delta_Robot_Package.dir_kin import *       # the direct kinematics of the delta robot
'''_______________________________________________________________________________________________________________________________________'''


# The Main Function:
def main():
   # Defining Arrays:
   n = 5

   x1 = np.full(n*n*n,None)
   y1 = np.full(n*n*n,None)
   z1 = np.full(n*n*n,None)

   x2 = np.full(n*n*n,None)
   y2 = np.full(n*n*n,None)
   z2 = np.full(n*n*n,None)
 

   i = 0
   for th1 in np.linspace(min_angle,max_angle,n):
      for th2 in np.linspace(min_angle,max_angle,n):
         for th3 in np.linspace(min_angle,max_angle,n):
            print(f'Loop {i}')

            v_thetas = np.array([th1,th2,th3])
            req_coords, error_coords = get_coords(v_thetas, solution='lower')
            x2[i],y2[i],z2[i] = req_coords
            
            ref_coords, error_num_coords = get_coords_numerically(v_thetas,initial_guess=np.array([x2[i],y2[i],z2[i]]))
            x1[i],y1[i],z1[i] = ref_coords

            if 0.001 < abs(100*(x1[i] - x2[i])):
               print(f"Error: different values for x")

            if 0.001 < abs(100*(y1[i] - y2[i])):
               print(f"Error: different values for y")

            if 0.001 < abs(100*(z1[i] - z2[i])):
               print(f"Error: different values for z")

            i += 1

   print(f"Done")

   plt.figure('Coordinates', layout='constrained')

   plt.subplot(311)
   plt.plot(np.arange(0,n*n*n,1), x1, color='black')
   plt.plot(np.arange(0,n*n*n,1), x2, color='red')
   plt.xlabel('iteration')
   plt.ylabel('X')

   plt.subplot(312)
   plt.plot(np.arange(0,n*n*n,1), y1, color='black')
   plt.plot(np.arange(0,n*n*n,1), y2, color='red')
   plt.xlabel('iteration')
   plt.ylabel('Y')

   plt.subplot(313)
   plt.plot(np.arange(0,n*n*n,1), z1, color='black')
   plt.plot(np.arange(0,n*n*n,1), z2, color='red')
   plt.xlabel('iteration')
   plt.ylabel('Z')

   plt.show()
'''_______________________________________________________________________________________________________________________________________'''


# Running the Main Function:
'''Note: "if __name__ == '__main__':" is a method used to ensure that this code does not run if imported to another file'''
if __name__ == '__main__':
    main()