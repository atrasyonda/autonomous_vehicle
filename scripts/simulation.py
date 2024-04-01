#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import random
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
from function import Kinematic
from constants import *

import time


def path_generator():
    # Plot the reference trajectory
    t=np.arange(0,10+Tc,Tc) # duration of the entire manoeuvre
    lane_width=7 # [m]
    r=8
    f=0.01
    x_dot = 20
    
    # Define the x length, depends on the car's longitudinal velocity
    x=np.linspace(0,x_dot*t[-1],num=len(t))
    statesTotal=np.zeros((len(t),n)) # It will keep track of all your states during the entire manoeuvre
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

    # Inisialisasi buffer dengan N elemen pertama dari data (N = horizon prediction)
    buffer_size = N
    Xr_dot = [dx[i:i+buffer_size] for i in range(len(dx) - buffer_size + 1)]
    Psi_dot = [dpsi[i:i+buffer_size] for i in range(len(dpsi) - buffer_size + 1)]
    # ====== PLOT TRAJECTORY =====
    # plt.plot(x,y,'b',linewidth=2,label='The trajectory')
    # # plt.plot(x,statesTotal[:,3],'--r',linewidth=2,label='Car position')
    # plt.xlabel('x-position [m]',fontsize=15)
    # plt.ylabel('y-position [m]',fontsize=15)
    # plt.grid(True)
    # plt.legend(loc='upper right',fontsize='small')
    # plt.ylim(-x[-1]/2,x[-1]/2) # Scale roads (x & y sizes should be the same to get a realistic picture of the situation)
    # plt.show()
    return x,y,psiInt,Xr_dot,Psi_dot


Ac_pk, Bc = Kinematic.getModel()  # get model parameters
P, Ki, S= Kinematic.getMPCSet(Ac_pk,Bc) 
# set initial variabel
a = 0

def openloop_control (data):
    
    x_k = np.array([[data.x], [data.y], [data.psi]])
    u_k = np.array([[data.delta], [a]])
    x_dot_psi_e = [x * data.psi for x in data.x_dot_ref]
    
    rc_k = np.array([[x_dot_psi_e], [data.psi_dot_ref]])
    # print (rc_k.shape)

    # print("state : ", x_k.shape)
    Ac = Kinematic.getLPV(data, Ac_pk)  # get LPV model
    x_opt, deltau_opt, u_opt = Kinematic.MPC(x_k, u_k, rc_k, Ac, Bc, P, S)

    # ======= Calculate next state ========
    control_signal = u_opt[0,:]
    next_state = Ac@x_k + Bc@control_signal - Bc@rc_k[:,:,0]
    # print("next_x_opt : ", x_opt[1])
    print("=====================================")
    print("next_state : ", next_state)
    print("control_signal : ", control_signal)
    return next_state, control_signal
    # pub.publish(state(x = next_state[0,0], y = next_state[1,0], psi = next_state[2,0], 
    #                   x_dot = control_signal[0,0], psi_dot = control_signal[0,0]))


if __name__=='__main__':
    X_r, Y_r, Psi_r, xr_dot, psi_r_dot = path_generator()
    car_pos_x = [] 
    car_pos_y= [] 
    car_psi = []

    ref_pos_x = []
    ref_pos_y = []
    ref_psi = []
    for i in range(len(xr_dot)):
        print  ("%d th loop" %i)
        car = state()
        if i == 0 :
            X_k = 0
            Y_k = 0
            Psi_k = 0
            x_dot = 0
            psi_dot = 0
        else : 
            X_k = X_r[i] - next_state[0,0]
            Y_k = Y_r[i] - next_state[1,0]
            Psi_k = Psi_r[i] - next_state[2,0]
            x_dot = control_signal[0,0]
            psi_dot = control_signal[1,0]

        ref_pos_x.append(X_r[i])
        ref_pos_y.append(Y_r[i])
        ref_psi.append(Psi_r[i])

        car_pos_x.append(X_k)
        car_pos_y.append(Y_k)
        car_psi.append(Psi_k)


        # === kinematic control =====
        car.x = X_r[i] - X_k
        car.y = Y_r[i] - Y_k
        car.psi = Psi_r[i] - Psi_k
        # === kinematic reference =====
        car.x_dot_ref = xr_dot[i]
        car.psi_dot_ref = psi_r_dot[i] 

        print("===========================")
        print("X_eror : ",X_r[i], " - ", X_k, " = ", car.x)
        print("Y_error : ", Y_r[i], " - ", Y_k, " = ", car.y)
        print("Psi_error : ",Psi_r[i], " - ", Psi_k, " = ", car.psi)
        print("Vd  : ", car.x_dot_ref)
        print("W omega : ", car.psi_dot_ref)
        print("===========================")
    

        next_state, control_signal = openloop_control(car)

        # time.sleep(1)
        i = i+1

    print("Done")
    # print("Car Position (m)")
    # print("X: ", car_pos_x)
    # print("Y: ", car_pos_y)
    # print("Angle : ", car_psi)
    print("Dimensi X", len(car_pos_x))
    print("Dimensi X_ref", len(ref_pos_x))
    print("Dimensi Y", len(car_pos_y))
    print("Dimensi Y_ref", len(ref_pos_y))
    print("Dimensi Psi", len(car_psi))
    print("Dimensi Psi_ref", len(ref_psi))

    # ====== PLOT TRAJECTORY =====
    plt.plot(ref_pos_x,ref_pos_y,'b',linewidth=2,label='The trajectory')
    plt.plot(car_pos_x,car_pos_y,'--r',linewidth=2,label='Car position')
    plt.xlabel('x-position [m]',fontsize=15)
    plt.ylabel('y-position [m]',fontsize=15)
    plt.grid(True)
    plt.legend(loc='upper right',fontsize='small')
    plt.ylim(-ref_pos_x[-1]/2,ref_pos_x[-1]/2) # Scale roads (x & y sizes should be the same to get a realistic picture of the situation)
    plt.show()




