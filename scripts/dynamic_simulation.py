#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib as plt
import random
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
from constants import *
from function import Dynamic
import time

Ad_vk, Bd = Dynamic.getModel()
Ki = Dynamic.getLQR(Ad_vk, Bd)

# Definisi list Vx dan Omega
X_dot = [
    19.99999984, 20.00000005, 19.8237829, 19.38813482, 18.04090925,
    16.6234485, 14.99752278, 13.51446888, 12.35752373, 11.59436772,
    11.26298749, 11.35599561, 11.85404882, 12.66373474, 13.52055195,
    14.26517241, 14.82150553, 15.16582077, 15.32669411, 15.363547,
    15.33063039, 15.27391544, 15.21783781, 15.17350469, 15.14132778,
    15.11784415, 15.10065365, 15.08742374, 15.07642448, 15.06898896,
    15.0618543, 15.05498, 15.04845413, 15.04238568, 15.03367611,
    15.02604094, 15.01953629, 15.01421024, 15.01010287, 15.00718857,
    15.00535517, 15.00442106, 15.00417701, 15.00442811, 15.00501708,
    15.00582259, 15.00674012, 15.00766162, 15.00764755, 15.00559542,
    15.00371754, 15.00202811, 15.00050676, 14.99912371, 14.99786165,
    14.99673679, 14.99634222, 14.99620443, 14.99601754, 14.99600257,
    14.99626253, 14.99675381, 14.99737947, 14.99867647, 14.99966073,
    15.00030005, 15.0006332, 15.00074078, 15.00071373, 15.00062977,
    15.00054187, 15.00047714, 15.00044242, 15.00043214, 15.0004356,
    15.0004422, 15.00044415, 15.00004373, 15.00004206, 15.00003954,
    15.00003641
]

Psi_dot = [
    -0.98799467, -1.1200002, -1.11999898, -1.1070488, -1.09847723,
    -1.25735414, -1.00351754, -0.68487412, -0.36164925, -0.02812339,
    0.31005554, 0.66147111, 0.99694101, 1.1572269, 1.2036958,
    1.17403432, 1.07073083, 0.90280159, 0.7233507, 0.54328679,
    0.37232614, 0.23277494, 0.12523621, 0.05115461, 0.00302654738,
    -0.0280988, -0.04500886, -0.05033875, -0.0460832, -0.02606532,
    -0.00131306415, 0.02703552, 0.05841002, 0.09257524, 0.11935518,
    0.15326292, 0.19320961, 0.23724214, 0.28285548, 0.3270622,
    0.366546, 0.39789688, 0.41791035, 0.4239043, 0.41400495,
    0.38735651, 0.34422723, 0.286005, 0.21251473, 0.1246131,
    0.03457225, -0.05416084, -0.13812178, -0.21407166, -0.27923928,
    -0.33153212, -0.36801869, -0.39040101, -0.40047687, -0.39882026,
    -0.3866915, -0.36584853, -0.33815397, -0.30348753, -0.26708721,
    -0.2307464, -0.19587978, -0.16351795, -0.13433199, -0.10867824,
    -0.08665416, -0.06815779, -0.05294514, -0.04068166, -0.0309856,
    -0.02346244, -0.01773032, -0.0134374876, -0.0102727407, -0.0079702536,
    -0.00631017991
]


def openloop_dynamic_control(data, a, reference):
    setpoint = []
    next20_states = []
    next20_inputs = []
    for i in range(9):
        if i == 0:
            vk = [data.delta, data.x_dot, data.y_dot]
            X_d = np.array([[data.x_dot], [data.y_dot],[data.psi_dot]])
            a = a
        else :
            vk = [U_d[0], next_state[0], next_state[1]]
            X_d = np.array([[next_state[0,0]], [next_state[1,0]],[next_state[2,0]]])
            a = U_d[1]
        print("reference", reference)
        print("vk = ", vk)
        if not(delta_min <= vk[0] <= delta_max): print("delta out of bound")
        if not(x_dot_min <= vk[1] <= x_dot_max): print("x_dot out of bound")
        if not(-y_dot_max <= vk[2] <= y_dot_max): print("y_dot out of bound")
        if not(a_min <= a <= a_max): print("a out of bound")
        
        Ad, K_vk = Dynamic.LPV_LQR(vk, Ad_vk, Ki)
        U_d = - K_vk @ X_d 

        # print("K_vk", K_vk) 
        # print("X_d", X_d)
        print("U_d", U_d)
        # print("U_d", U_d.shape)
        # print("Bd", Bd.shape)
        #  Calculate Next States
        next_state = Ad @ X_d + Bd @ U_d
        
        setpoint.append(reference)
        next20_states.append(X_d)
        next20_inputs.append(U_d)
        print ("=========================================")

    # print ("setpoint", len(setpoint))
    # print ("next20_states", len(next20_states))  
    # print ("next20_inputs", len(next20_inputs))  
    
    return next20_states, next20_inputs

if __name__=='__main__':
    all_state = []
    
    for i in range(1):
        print (" %d th loop" %i)
        print ("==============")
        car = state()
        reference = np.array([[X_dot[i+1]], [Psi_dot[i+1]]])

        Vx = X_dot[i]
        omega = Psi_dot[i]
        if i == 0 :
            Vy = 0
            delta = 0
            a = 0
        else :
            Vy = next20_state[-1][0][0]
            delta = next20_input[-1][0][0]
            a = next20_input[-1][1][0]

        car.x_dot = Vx
        car.y_dot = Vy
        car.psi_dot = omega
        car.delta = delta

        # print("Vx", Vx)
        # print("Vy", Vy)
        # print("omega", omega)
        # print("delta", delta)
        # print("a", a)

        # print ("reference", reference)
        print ("==============")
        next20_state, next20_input = openloop_dynamic_control(car,a, reference)

        print ("==============")
        i = i+1
        

