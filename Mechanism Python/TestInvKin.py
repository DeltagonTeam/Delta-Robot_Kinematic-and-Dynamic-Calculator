'''
==========================================================================================================================
This tests the inverse kinematics of the Delta-Robot
==========================================================================================================================
'''


# Importing:
'''Note: "from ... import *": imports all names from the file except for ones that start with underscore (conventionally private)'''
import matplotlib.pyplot as plt     # for plotting
import numpy as np      # for arrays and matrix operations
from Delta_Robot_Package.constants import *     # the dimentional constants of the delta robot
from Delta_Robot_Package.inv_kin import *       # the inverse kinematics of the delta robot
from Delta_Robot_Package.ref_kin import *       # the reference inverse kinematics of the delta robot
from Delta_Robot_Package.end_effector_path import *     # the end effector path
'''_______________________________________________________________________________________________________________________________________'''


# The Main Function:
def main():
   # Defining Arrays:
   n = 70         # number of loops
   t_start = 6    # starting time
   t_stop = 13    # time step size
   arr_t = np.linspace(t_start, t_stop, n)      #defining the time array


   v_ref_th_1a = np.full(n,None)
   v_ref_th_2a = np.full(n,None)
   v_ref_th_3a = np.full(n,None)
   v_ref_th_1b = np.full(n,None)
   v_ref_th_2b = np.full(n,None)
   v_ref_th_3b = np.full(n,None)
   v_ref_th_1 = np.full(n,None)
   v_ref_th_2 = np.full(n,None)
   v_ref_th_3 = np.full(n,None)

   v_ref_thd_1 = np.full(n,None)
   v_ref_thd_2 = np.full(n,None)
   v_ref_thd_3 = np.full(n,None)

   v_ref_thdd_1 = np.full(n,None)
   v_ref_thdd_2 = np.full(n,None)
   v_ref_thdd_3 = np.full(n,None)


   v_req_th_1 = np.full(n,None)
   v_req_th_2 = np.full(n,None)
   v_req_th_3 = np.full(n,None)

   v_req_thd_1 = np.full(n,None)
   v_req_thd_2 = np.full(n,None)
   v_req_thd_3 = np.full(n,None)

   v_req_thdd_1 = np.full(n,None)
   v_req_thdd_2 = np.full(n,None)
   v_req_thdd_3 = np.full(n,None)

   for i in np.arange(0, n, 1):
      print(f"Loop:{i}\nTime:{arr_t[i]}")

      if (0 > ref_under_sqrt1.subs([(t,arr_t[i])]).evalf()) or (0 > ref_under_sqrt2.subs([(t,arr_t[i])]).evalf()) or (0 > ref_under_sqrt3.subs([(t,arr_t[i])]).evalf()):
         print("Error: coordinates not possible")
         continue
      
      v_x = path_x.subs([(t,arr_t[i])]).evalf()
      v_xd = path_xd.subs([(t,arr_t[i])]).evalf()
      v_xdd = path_xdd.subs([(t,arr_t[i])]).evalf()

      v_y = path_y.subs([(t,arr_t[i])]).evalf()   
      v_yd = path_yd.subs([(t,arr_t[i])]).evalf()
      v_ydd = path_ydd.subs([(t,arr_t[i])]).evalf()

      v_z = path_z.subs([(t,arr_t[i])]).evalf()
      v_zd = path_zd.subs([(t,arr_t[i])]).evalf()
      v_zdd = path_zdd.subs([(t,arr_t[i])]).evalf()
      
      #Comparing each calculated value with the reference value(gotten with sympy):
      v_ref_th_1a[i] = ref_th_1a.subs([(t,arr_t[i])]).evalf()
      v_ref_th_2a[i] = ref_th_2a.subs([(t,arr_t[i])]).evalf()
      v_ref_th_3a[i] = ref_th_3a.subs([(t,arr_t[i])]).evalf()
      
      v_ref_th_1b[i] = ref_th_1b.subs([(t,arr_t[i])]).evalf()
      v_ref_th_2b[i] = ref_th_2b.subs([(t,arr_t[i])]).evalf()
      v_ref_th_3b[i] = ref_th_3b.subs([(t,arr_t[i])]).evalf()

      if (abs(v_ref_th_1a[i]) < abs(v_ref_th_1b[i])):
         v_ref_th_1[i] = v_ref_th_1a[i]
         v_ref_thd_1[i] = ref_thd_1a.subs([(t,arr_t[i])]).evalf()
         v_ref_thdd_1[i] = ref_thdd_1a.subs([(t,arr_t[i])]).evalf()
      else:
         v_ref_th_1[i] = v_ref_th_1b[i]
         v_ref_thd_1[i] = ref_thd_1b.subs([(t,arr_t[i])]).evalf()
         v_ref_thdd_1[i] = ref_thdd_1b.subs([(t,arr_t[i])]).evalf()

      if (abs(v_ref_th_2a[i]) < abs(v_ref_th_2b[i])):
         v_ref_th_2[i] = v_ref_th_2a[i]
         v_ref_thd_2[i] = ref_thd_2a.subs([(t,arr_t[i])]).evalf()
         v_ref_thdd_2[i] = ref_thdd_2a.subs([(t,arr_t[i])]).evalf()
      else:
         v_ref_th_2[i] = v_ref_th_2b[i]
         v_ref_thd_2[i] = ref_thd_2b.subs([(t,arr_t[i])]).evalf()
         v_ref_thdd_2[i] = ref_thdd_2b.subs([(t,arr_t[i])]).evalf()

      if (abs(v_ref_th_3a[i]) < abs(v_ref_th_3b[i])):
         v_ref_th_3[i] = v_ref_th_3a[i]
         v_ref_thd_3[i] = ref_thd_3a.subs([(t,arr_t[i])]).evalf()
         v_ref_thdd_3[i] = ref_thdd_3a.subs([(t,arr_t[i])]).evalf()
      else:
         v_ref_th_3[i] = v_ref_th_3b[i]
         v_ref_thd_3[i] = ref_thd_3b.subs([(t,arr_t[i])]).evalf()
         v_ref_thdd_3[i] = ref_thdd_3b.subs([(t,arr_t[i])]).evalf()

      v_coords = np.array([v_x,v_y,v_z])
      v_velocities =  np.array([v_xd,v_yd,v_zd])
      v_accelerations =  np.array([v_xdd,v_ydd,v_zdd])

      v_req_thetas, error = get_thetas(v_coords)
      v_req_th_1[i],v_req_th_2[i],v_req_th_3[i] =  v_req_thetas
      
      v_req_theta_dots = get_theta_dots(v_coords, v_velocities, v_req_thetas)
      v_req_thd_1[i],v_req_thd_2[i],v_req_thd_3[i] = v_req_theta_dots

      v_req_theta_double_dots = get_theta_double_dots(v_coords, v_velocities, v_accelerations, v_req_thetas, v_req_theta_dots)
      v_req_thdd_1[i],v_req_thdd_2[i],v_req_thdd_3[i] = v_req_theta_double_dots

      if 0.001 < abs((100*(v_ref_th_1[i] - v_req_th_1[i]) / v_ref_th_1[i])):
         print(f"Error: different values for th_1")
      if 0.001 < abs((100*(v_ref_th_2[i] - v_req_th_2[i]) / v_ref_th_2[i])):
         print(f"Error: different values for th_2")
      if 0.001 < abs((100*(v_ref_th_3[i] - v_req_th_3[i]) / v_ref_th_3[i])):
         print(f"Error: different values for th_3")

      if 0.001 < abs((100*(v_ref_thd_1[i] - v_req_thd_1[i]) / v_ref_thd_1[i])):
         print(f"Error: different values for thd_1")
      if 0.001 < abs((100*(v_ref_thd_2[i] - v_req_thd_2[i]) / v_ref_thd_2[i])):
         print(f"Error: different values for thd_2")
      if 0.001 < abs((100*(v_ref_thd_3[i] - v_req_thd_3[i]) / v_ref_thd_3[i])):
         print(f"Error: different values for thd_3")

      if 0.001 < abs((100*(v_ref_thdd_1[i] - v_req_thdd_1[i]) / v_ref_thdd_1[i])):
         print(f"Error: different values for thdd_1")
      if 0.001 < abs((100*(v_ref_thdd_2[i] - v_req_thdd_2[i]) / v_ref_thdd_2[i])):
         print(f"Error: different values for thdd_2")
      if 0.001 < abs((100*(v_ref_thdd_3[i] - v_req_thdd_3[i]) / v_ref_thdd_3[i])):
         print(f"Error: different values for thdd_3")

   plt.figure('Theta 1', layout='constrained')
   plt.subplot(311)
   plt.plot(arr_t, v_ref_th_1, color='black')
   plt.plot(arr_t, v_req_th_1, color='red')
   plt.xlabel('time')
   plt.ylabel('theta 1')
   plt.legend(['Reference', 'Calculated'])
   plt.subplot(312)
   plt.plot(arr_t, v_ref_thd_1, color='black')
   plt.plot(arr_t, v_req_thd_1, color='red')
   plt.xlabel('time')
   plt.ylabel('omega 1')
   plt.legend(['Reference', 'Calculated'])
   plt.subplot(313)
   plt.plot(arr_t, v_ref_thdd_1, color='black')
   plt.plot(arr_t, v_req_thdd_1, color='red')
   plt.xlabel('time')
   plt.ylabel('alpha 1')
   plt.legend(['Reference', 'Calculated'])

   plt.figure('Theta 2', layout='constrained')
   plt.subplot(311)
   plt.plot(arr_t, v_ref_th_2, color='black')
   plt.plot(arr_t, v_req_th_2, color='red')
   plt.xlabel('time')
   plt.ylabel('theta 2')
   plt.legend(['Reference', 'Calculated'])
   plt.subplot(312)
   plt.plot(arr_t, v_ref_thd_2, color='black')
   plt.plot(arr_t, v_req_thd_2, color='red')
   plt.xlabel('time')
   plt.ylabel('omega 2')
   plt.legend(['Reference', 'Calculated'])
   plt.subplot(313)
   plt.plot(arr_t, v_ref_thdd_2, color='black')
   plt.plot(arr_t, v_req_thdd_2, color='red')
   plt.xlabel('time')
   plt.ylabel('alpha 2')
   plt.legend(['Reference', 'Calculated'])

   plt.figure('Theta 3', layout='constrained')
   plt.subplot(311)
   plt.plot(arr_t, v_ref_th_3, color='black')
   plt.plot(arr_t, v_req_th_3, color='red')
   plt.xlabel('time')
   plt.ylabel('theta 3')
   plt.legend(['Reference', 'Calculated'])
   plt.subplot(312)
   plt.plot(arr_t, v_ref_thd_3, color='black')
   plt.plot(arr_t, v_req_thd_3, color='red')
   plt.xlabel('time')
   plt.ylabel('omega 3')
   plt.legend(['Reference', 'Calculated'])
   plt.subplot(313)
   plt.plot(arr_t, v_ref_thdd_3, color='black')
   plt.plot(arr_t, v_req_thdd_3, color='red')
   plt.xlabel('time')
   plt.ylabel('alpha 3')
   plt.legend(['Reference', 'Calculated'])

   plt.show()
'''_______________________________________________________________________________________________________________________________________'''


# Running the Main Function:
'''Note: "if __name__ == '__main__':" is a method used to ensure that this code does not run if imported to another file'''
if __name__ == '__main__':
    main()