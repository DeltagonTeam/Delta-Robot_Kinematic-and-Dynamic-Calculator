# Importing:
import matplotlib.pyplot as plt
import math
import numpy as np


# Defining Variables:
    # Configurations:
n = 1000
L = 0.024
R = 1.8
I0 = 3
tmax = 0.1
rpm_max = 100
steps_per_rev = 200

    # Arrays for Plotting:
Ion = np.full(n, None)      # the current while turning the coil on in mA
Ioff = np.full(n, None)     # the current while turning the coil off in mA
Imax = np.full(n, None)
rpm = np.linspace(0,rpm_max,n)
t = np.linspace(0,tmax,n)   # time in s


# Defining Functions:
def Current_CoilOn(R, L, t):
    Tau = L/R
    return 100*(1-math.exp(-t/Tau))

def Current_CoilOff(R, L, t):
    Tau = L/R
    return 100*(math.exp(-t/Tau))

def Current_Max(R, L, rpm, step_per_rev):
    t_step = (60/rpm)/step_per_rev
    return Current_CoilOn(R, L, t_step)


# The Main Function:
def main():
    # Calculating Current:
    for i in np.arange(start=0, stop=n, step=1):
        Ion[i] = Current_CoilOn(R, L, t[i])
        Ioff[i] = Current_CoilOff(R, L, t[i])
        Imax[i] = Current_Max(R, L, rpm[i], steps_per_rev)

    # Plotting:
    plt.figure(f'Turning The Coil On', layout='constrained')
    plt.plot(t,Ion,color='Darkcyan')
    plt.xlabel('time (ms)')
    plt.ylabel('Current (%)')

    plt.figure(f'Turning The Coil Off', layout='constrained')
    plt.plot(t,Ioff,color='Darkcyan')
    plt.xlabel('time (ms)')
    plt.ylabel('Current (%)')

    plt.figure(f'Current with RPM', layout='constrained')
    plt.plot(rpm,Imax,color='Darkcyan')
    plt.xlabel('Speed (rpm)')
    plt.ylabel('Max Current (%)')

    plt.show()


# Running the Main Function:
'''Note: "if __name__ == '__main__':" is a method used to ensure that this code does not run if imported to another file'''
if __name__ == '__main__':
    main()