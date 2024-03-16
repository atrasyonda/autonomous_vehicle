import numpy as np
from function import Kinematic
# from constants import *

def trajectory_generator():
    '''This method creates the trajectory for a car to follow'''
    Tc=0.1
    # time from 0 to 10 seconds, sample time (Tc=0.1 second)
    t=np.arange(0,10+Tc,Tc) 
    r=4 # amplitudo sinusoidal trajectory
    f= 0.01 # frequency
    
    x_dot=20 # car velocity constant
    lane_width=7

    # Define the x length, depends on the car's longitudinal velocity
    x=np.linspace(0,x_dot*t[-1],num=len(t))

    # Define trajectories
    aaa=-28/100**2
    aaa=aaa/1.1
    if aaa<0:
        bbb=14
    else:
        bbb=-14
    y_1=aaa*(x+lane_width-100)**2+bbb
    y_2=2*r*np.sin(2*np.pi*f*x)
    y=(y_1+y_2)/2

    # Vector of x and y changes per sample time
    dx=x[1:len(x)]-x[0:len(x)-1]
    dy=y[1:len(y)]-y[0:len(y)-1]

    # Define the reference yaw angles
    psi=np.zeros(len(x))
    psiInt=psi
    psi[0]=np.arctan2(dy[0],dx[0])
    psi[1:len(psi)]=np.arctan2(dy[0:len(dy)],dx[0:len(dx)])

    # We want the yaw angle to keep track the amount of rotations
    dpsi=psi[1:len(psi)]-psi[0:len(psi)-1]
    psiInt[0]=psi[0]
    for i in range(1,len(psiInt)):
        if dpsi[i-1]<-np.pi:
            psiInt[i]=psiInt[i-1]+(dpsi[i-1]+2*np.pi)
        elif dpsi[i-1]>np.pi:
            psiInt[i]=psiInt[i-1]+(dpsi[i-1]-2*np.pi)
        else:
            psiInt[i]=psiInt[i-1]+dpsi[i-1]
    return psiInt,x,y

outputs = 3 # x,y, psi
# Generate the refence signals
psi_ref,X_ref,Y_ref=trajectory_generator()
sim_length=len(t) # Number of control loop iterations
refSignals=np.zeros(len(X_ref)*outputs)

# Build up the reference signal vector:
# refSignal = [psi_ref_0, Y_ref_0, psi_ref_1, Y_ref_1, psi_ref_2, Y_ref_2, ... etc.]
k=0
for i in range(0,len(refSignals),outputs):
    refSignals[i]=psi_ref[k]
    refSignals[i+1]=Y_ref[k]
    k=k+1


# Load the initial states
# If you want to put numbers here, please make sure that they are float and not
# integers. It means that you should add a point there.
# Example: Please write 0. in stead of 0 (Please add the point to make it float)
y_dot=0.
psi=0.
psi_dot=0.
Y=Y_ref[0]+10.

states=np.array([y_dot,psi,psi_dot,Y])
statesTotal=np.zeros((len(t),len(states))) # It will keep track of all your states during the entire manoeuvre
statesTotal[0][0:len(states)]=states
psi_opt_total=np.zeros((len(t),hz))
Y_opt_total=np.zeros((len(t),hz))

# Load the initial input
U1=0 # Input at t = -1 s (steering wheel angle in rad (delta))
UTotal=np.zeros(len(t)) # To keep track all your inputs over time
UTotal[0]=U1

