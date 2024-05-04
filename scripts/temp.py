#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import random
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
from constants import *
from function import Dynamic,Kinematic
from lmi_param import K_d

Ad_vk, Bd = Dynamic.getModel()
Ac_pk, Bc = Kinematic.getModel()  # get model parameters
# print("Ac_pk", Ac_pk)
# print("Ad_vk",Ad_vk)

# ================ DYNAMIC EIGEN & RESPONSE TEST =================
# delta = np.linspace(delta_min, delta_max, 10)
# x_dot = np.linspace(x_dot_min, x_dot_max, 10)
# y_dot = np.linspace(y_dot_min, y_dot_max, 10)

# psi_dot = np.linspace(psi_dot_min, psi_dot_max, 10)

# openloop_eigen = np.zeros([10,3])
# cleseloop_eigen = np.zeros([10,3])

# reference = np.array([[-1], [19]]) # [Psidot , Vx]
# Kr = 1
# # print(reference)

# for i in range(10):
#     vk = [delta[i], x_dot[i], y_dot[i]]
#     X_d = np.array([[x_dot[i]], [y_dot[i]],[psi_dot[i]]])
#     Ad, K_vk, Miu_vk = Dynamic.LPV_LQR(vk, Ad_vk, K_d)
#     # print("Loop Ke -", i)
#     # print("=========================================")
#     # print(Ad)
#     # print("=========================================")
#     # print(K_vk)
#     print("=========================================")
#     eigenvalue = np.linalg.eigvals(Ad)
#     print("Open-loop Eigenvalue", eigenvalue)
#     openloop_eigen[i] = eigenvalue
#     eigenvalue = np.linalg.eigvals(Ad-Bd@K_vk)
#     cleseloop_eigen[i] = eigenvalue
#     print("Closed-loop Eigenvalue", eigenvalue)
#     print("=========================================")

# print("Open-loop Eigenvalue", openloop_eigen)
# print("Closed-loop Eigenvalue", cleseloop_eigen)


# ========== SIMULASI DENGAN ERROR STATE [XDOT, YDOT, PSIDOT] ==========

Xr_dot = [
    19.99999984, 19.99999984, 19.8237829
]

Psi_r_dot = [
    -0.98799467, -1.1200002, -1.11999898
]

def dynamic_control(data, a, reference):
    Kr = 1
    iteration = []
    Error = []
    n = 3
    next20_states = np.zeros((n,3,1))
    next20_inputs = np.zeros((n,2,1))
    print("reference", reference)
    for i in range(n):
        print("Loop Ke -", i)
        print ("=========================================")
        if i == 0:
            error = reference*Kr - np.array([[data.x_dot], [data.y_dot], [data.psi_dot]])
            print("Error Setpoint", error)
            vk = [data.delta, data.x_dot, data.y_dot]
            # X_d = error
            X_d = np.array([[data.x_dot], [data.y_dot],[data.psi_dot]])
            a = a
        else : 
            error = reference*Kr - np.array([[data.x_dot], [data.y_dot], [data.psi_dot]])
            print("Error Setpoint", error)
            vk = [data.delta, data.x_dot, data.y_dot]
            # X_d = error
            X_d = np.array([[data.x_dot], [data.y_dot],[data.psi_dot]])
            a = a
        
        if not(delta_min <= vk[0] <= delta_max): print("delta out of bound")
        if not(x_dot_min <= vk[1] <= x_dot_max): print("x_dot out of bound")
        if not(-y_dot_max <= vk[2] <= y_dot_max): print("y_dot out of bound")
        if not(a_min <= a <= a_max): print("a out of bound")
        
        Ad, K_vk, Miu_vk = Dynamic.LPV_LQR(vk, Ad_vk, K_d)
        U_d = -K_vk @ X_d
        print("K_vk", K_vk) 
        print("X_d", X_d)
        print ("=========================================")
        eig_Acl = np.linalg.eigvals(Ad-Bd@K_vk)
        print("Eigenvalues of A-BK", eig_Acl)
        if np.any(eig_Acl > 0):
            print("System is Unstable")
        else:
            print("System is Stable")

        print("U_d", U_d)
        # next_state = Ad @ X_d + Bd @ U_d
        next_state = (Ad-Bd@K_vk) @ X_d
        print("Next Dynamic State",next_state)
        output = next_state 
        print("Output", output)

        data.x_dot = output[0,0]
        data.y_dot = output[1,0]
        data.psi_dot = output[2,0]
        data.delta = U_d[0,0]
        a = U_d[1,0]

        iteration.append(i)
        # Error.append(error)
        next20_states[i] = X_d
        next20_inputs[i] = U_d
        print ("=========================================")
        

    # print ("Error", len(error))
    print ("next20_states", len(next20_states))  
    print ("next20_inputs", len(next20_inputs))  
    
    return iteration, next20_states, next20_inputs

if __name__=='__main__':
    all_state = []
    for i in range(1):
        print (" %d th loop" %i)
        print ("==============")
        car = state()
        reference = np.array([[Xr_dot[i]], [0], [Psi_r_dot[i]]])
        if i == 0 : # initial condition
            Vx = 0.1
            Vy = 0
            Psi_dot = 0
            delta = 0
            a = 0
        car.x_dot = Vx
        car.y_dot = Vy
        car.psi_dot = Psi_dot
        car.delta = delta
        print ("===== CALCULATE DYNAMIC =========")
        iteration, next20_state, next20_input = dynamic_control(car,a, reference)
        print ("==============")
        i = i+1
        
        # print("Next 20 State", next20_state) # Vx,Vy,Psi_dot
        # print("Next 20 Input", next20_input) # delta, a
    Vx_upperbound = [x_dot_max for i in range(n)]
    Vx_lowerbound = [x_dot_min for i in range(n)]

    W_upperbound = [psi_dot_max for i in range(n)]
    W_lowerbound = [psi_dot_min for i in range(n)]

    delta_upperbound = [delta_max for i in range(n)]
    delta_lowerbound = [delta_min for i in range(n)]

    a_upperbound = [a_max for i in range(n)]
    a_lowerbound = [a_min for i in range(n)]

    # Membuat subplot pertama
    plt.subplot(4, 1, 1)  # 3 baris, 1 kolom, subplot pertama
    plt.title('Longitudinal Velocity (m/s)')
    plt.plot(iteration,[Xr_dot[0] for i in range(n)],'--r',linewidth=2,label='Setpoint')
    plt.plot(iteration,next20_state[:,0,0],'b',linewidth=2,label='Car')
    plt.plot(iteration,Vx_upperbound,'r',linewidth=2,label='Upper Bound')
    plt.plot(iteration,Vx_lowerbound,'r',linewidth=2,label='Lower Bound')
    plt.xlabel('Iterasi')
    plt.ylabel('Vx')
    plt.legend(loc='upper right',fontsize='small')

    # Membuat subplot kedua
    plt.subplot(4, 1, 2)  # 3 baris, 1 kolom, subplot kedua
    plt.title('Angular Velocity (Rad/s)')
    plt.plot(iteration,[Psi_r_dot[0] for i in range(n)],'--r',linewidth=2,label='Setpoint')
    plt.plot(iteration,next20_state[:,2,0],'b',linewidth=2,label='Car')
    plt.plot(iteration,W_upperbound,'r',linewidth=2,label='Upper Bound')
    plt.plot(iteration,W_lowerbound,'r',linewidth=2,label='Lower Bound')
    plt.xlabel('Iterasi')
    plt.ylabel('Omega')
    plt.legend(loc='upper right',fontsize='small')


    # Membuat subplot ketiga
    plt.subplot(4, 1, 3)  # 3 baris, 1 kolom, subplot ketiga
    plt.title('Steering Angle (Rad)')
    plt.plot(iteration,next20_input[:,0,0],'b',linewidth=2,label='Car Steering')
    plt.plot(iteration,delta_upperbound,'r',linewidth=2,label='Upper Bound')
    plt.plot(iteration,delta_lowerbound,'r',linewidth=2,label='Lower Bound')
    plt.xlabel('Iterasi')
    plt.ylabel('delta')
    plt.legend(loc='upper right',fontsize='small')

    # Membuat subplot ketiga
    plt.subplot(4, 1, 4)  # 3 baris, 1 kolom, subplot ketiga
    plt.title('Acceleration (m/s^2)')
    plt.plot(iteration,next20_input[:,1,0],'b',linewidth=2,label='Car Acceleration')
    plt.plot(iteration,a_upperbound,'r',linewidth=2,label='Upper Bound')
    plt.plot(iteration,a_lowerbound,'r',linewidth=2,label='Lower Bound')
    plt.xlabel('Iterasi')
    plt.ylabel('a')
    plt.legend(loc='upper right',fontsize='small')

    plt.show()