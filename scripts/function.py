#!/usr/bin/env python3

#==============================
import rospy
import cvxpy as cp
import numpy as np
import matplotlib as plt
import random
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
from constants import *

class LPV:
    def __init__(self) -> None:
        pass
    def getKinematic(car:state):
        xr_dot = car.xr_dot
        psi_dot= car.psi_dot
        psi=car.psi

        # print("======================")
        # print(psi_dot)
        # print(xr_dot)
        # print(psi)
        # print("======================")

        nu0_psidot= (psi_dot_max-psi_dot)/(psi_dot_max-psi_dot_min)
        nu1_psidot= 1-nu0_psidot
        nu0_xrdot= (xr_dot_max-xr_dot)/(xr_dot_max-xr_dot_min)
        nu1_xrdot= 1-nu0_xrdot
        nu0_psi= (psi_max-psi)/(psi_max-psi_min)
        nu1_psi= 1-nu0_psi
        
        miu_pk = [
            [nu0_psidot*nu0_xrdot*nu0_psi],
            [nu0_psidot*nu0_xrdot*nu1_psi],
            [nu0_psidot*nu1_xrdot*nu0_psi],
            [nu0_psidot*nu1_xrdot*nu1_psi],
            [nu1_psidot*nu0_xrdot*nu0_psi],
            [nu1_psidot*nu0_xrdot*nu1_psi],
            [nu1_psidot*nu1_xrdot*nu0_psi],
            [nu1_psidot*nu1_xrdot*nu1_psi],
        ]

        A0=np.array([
            [1, psi_dot_min*Tc, 0],
            [-psi_dot_min*Tc, 1, xr_dot_min*np.sin(psi_min)*Tc/psi_min],
            [0, 0, 1]
        ])
        A1=np.array([
            [1, psi_dot_min*Tc, 0],
            [-psi_dot_min*Tc, 1, xr_dot_min*np.sin(psi_max)*Tc/psi_max],
            [0, 0, 1]
        ])
        A2=np.array([
            [1, psi_dot_min*Tc, 0],
            [-psi_dot_min*Tc, 1, xr_dot_max*np.sin(psi_min)*Tc/psi_min],
            [0, 0, 1]
        ])
        A3=np.array([
            [1, psi_dot_min*Tc, 0],
            [-psi_dot_min*Tc, 1, xr_dot_max*np.sin(psi_max)*Tc/psi_max],
            [0, 0, 1]
        ])
        A4=np.array([
            [1, psi_dot_max*Tc, 0],
            [-psi_dot_max*Tc, 1, xr_dot_min*np.sin(psi_min)*Tc/psi_min],
            [0, 0, 1]
        ])
        A5=np.array([
            [1, psi_dot_max*Tc, 0],
            [-psi_dot_max*Tc, 1, xr_dot_min*np.sin(psi_max)*Tc/psi_max],
            [0, 0, 1]
        ])
        A6=np.array([
            [1, psi_dot_max*Tc, 0],
            [-psi_dot_max*Tc, 1, xr_dot_max*np.sin(psi_min)*Tc/psi_min],
            [0, 0, 1]
        ])
        A7=np.array([
            [1, psi_dot_max*Tc, 0],
            [-psi_dot_max*Tc, 1, xr_dot_max*np.sin(psi_max)*Tc/psi_max],
            [0, 0, 1]
        ])
        Ac_pk = [A0, A1, A2, A3, A4, A5, A6, A7]

        Ac=0
        for i in range(8):
            Ac+=(Ac_pk[i]*miu_pk[i])
        Bc= np.array([
            [-Tc,0],
            [0,0],
            [0,-Tc]
        ])
        # log_message = "Nilai xr_dot: %s, Nilai psi_dot: %s, Nilai psi: %s" % (xr_dot, psi_dot,psi)
        # log_message = "Nilai Ac: %s, Nilai Bc: %s" % (Ac, Bc)
        # rospy.loginfo(log_message)
        # print("A0 : ", A0)
        # print("A1 : ", A1)
        # print("A2 : ", A2)
        # print("A3 : ", A3)
        # print("A4 : ", A4)
        # print("A5 : ", A5)
        # print("A6 : ", A6)
        # print("A7 : ", A7)
        return Ac,Ac_pk,Bc
    def getDynamic(car:state):
        x_dot = car.x_dot
        y_dot = car.y_dot
        delta = car.delta #sudut steering
        
        # print("======================")
        # print(delta)
        # print(x_dot)
        # print(y_dot)
        # print("======================")
        
        nu0_delta= (delta_max-delta)/(delta_max-delta_min)
        nu1_delta= 1-nu0_delta
        nu0_xdot= (x_dot_max-x_dot)/(x_dot_max-x_dot_min)
        nu1_xdot= 1-nu0_xdot
        nu0_ydot= (y_dot_max-y_dot)/(y_dot_max-y_dot_min)
        nu1_ydot= 1-nu0_ydot

        miu_vk = [
            [nu0_delta*nu0_xdot*nu0_ydot],
            [nu0_delta*nu0_xdot*nu1_ydot],
            [nu0_delta*nu1_xdot*nu0_ydot],
            [nu0_delta*nu1_xdot*nu1_ydot],
            [nu1_delta*nu0_xdot*nu0_ydot],
            [nu1_delta*nu0_xdot*nu1_ydot],
            [nu1_delta*nu1_xdot*nu0_ydot],
            [nu1_delta*nu1_xdot*nu1_ydot],
        ]
        
        A0=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_min**2 + miu*m*g)/ (m*x_dot_min))*Td, ((Caf*np.sin(delta_min))/(m*x_dot_min))*Td, (Caf*lf*np.sin(delta_min)/(m*x_dot_min)+y_dot_min)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_min))/(m*x_dot_min))*Td, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(m*x_dot_min)-x_dot_min)*Td],
            [0, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(I*x_dot_min))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_min)-Car*lr**2)/(I*x_dot_min))*Td]
        ])
        A1=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_min**2 + miu*m*g)/ (m*x_dot_min))*Td, ((Caf*np.sin(delta_min))/(m*x_dot_min))*Td, (Caf*lf*np.sin(delta_min)/(m*x_dot_min)+y_dot_max)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_min))/(m*x_dot_min))*Td, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(m*x_dot_min)-x_dot_min)*Td],
            [0, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(I*x_dot_min))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_min)-Car*lr**2)/(I*x_dot_min))*Td]
        ])
        A2=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_max**2 + miu*m*g)/ (m*x_dot_max))*Td, ((Caf*np.sin(delta_min))/(m*x_dot_max))*Td, (Caf*lf*np.sin(delta_min)/(m*x_dot_max)+y_dot_min)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_min))/(m*x_dot_max))*Td, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(m*x_dot_max)-x_dot_max)*Td],
            [0, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(I*x_dot_max))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_min)-Car*lr**2)/(I*x_dot_max))*Td]
        ])
        A3=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_max**2 + miu*m*g)/ (m*x_dot_max))*Td, ((Caf*np.sin(delta_min))/(m*x_dot_max))*Td, (Caf*lf*np.sin(delta_min)/(m*x_dot_max)+y_dot_max)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_min))/(m*x_dot_max))*Td, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(m*x_dot_max)-x_dot_max)*Td],
            [0, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(I*x_dot_max))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_min)-Car*lr**2)/(I*x_dot_max))*Td]
        ])
        A4=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_min**2 + miu*m*g)/ (m*x_dot_min))*Td, ((Caf*np.sin(delta_max))/(m*x_dot_min))*Td, (Caf*lf*np.sin(delta_max)/(m*x_dot_min)+y_dot_min)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_max))/(m*x_dot_min))*Td, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(m*x_dot_min)-x_dot_min)*Td],
            [0, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(I*x_dot_min))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_max)-Car*lr**2)/(I*x_dot_min))*Td]
        ])
        A5=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_min**2 + miu*m*g)/ (m*x_dot_min))*Td, ((Caf*np.sin(delta_max))/(m*x_dot_min))*Td, (Caf*lf*np.sin(delta_max)/(m*x_dot_min)+y_dot_max)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_max))/(m*x_dot_min))*Td, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(m*x_dot_min)-x_dot_min)*Td],
            [0, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(I*x_dot_min))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_max)-Car*lr**2)/(I*x_dot_min))*Td]
        ])
        A6=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_max**2 + miu*m*g)/ (m*x_dot_max))*Td, ((Caf*np.sin(delta_max))/(m*x_dot_max))*Td, (Caf*lf*np.sin(delta_max)/(m*x_dot_max)+y_dot_min)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_max))/(m*x_dot_max))*Td, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(m*x_dot_max)-x_dot_max)*Td],
            [0, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(I*x_dot_max))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_max)-Car*lr**2)/(I*x_dot_max))*Td]
        ])
        A7=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_max**2 + miu*m*g)/ (m*x_dot_max))*Td, ((Caf*np.sin(delta_max))/(m*x_dot_max))*Td, (Caf*lf*np.sin(delta_max)/(m*x_dot_max)+y_dot_max)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_max))/(m*x_dot_max))*Td, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(m*x_dot_max)-x_dot_max)*Td],
            [0, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(I*x_dot_max))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_max)-Car*lr**2)/(I*x_dot_max))*Td]
        ])
        
        Ad_vk=[A0,A1,A2,A3,A4,A5,A6,A7]
        Ad=0
        for i in range(8):
            Ad+=(Ad_vk[i]*miu_vk[i])
        
        # print (Ad)
        Bd= np.array([
            [0, Td],
            [Td*Caf/m, 0],
            [Td*Caf*lf/I, 0]
        ])
        # log_message = "Nilai x_dot: %s, Nilai y_dot: %s, Nilai delta: %s" % (x_dot, y_dot,delta)
        # rospy.loginfo(log_message)
        return Ad,Ad_vk,Bd,miu_vk

class LMI:
    def __init__(self) -> None:
        pass
    def getMPCSet(Ac_pk,Bc):
        n = 3  # number of states
        m = 2  # number of inputs
        invRts= np.linalg.inv(Rts)
        invQts= np.linalg.inv(Qts)
        Y = cp.Variable((n, n),symmetric=True)
        Wi = cp.Variable((m, n)) # Solusi bobot untuk kontroler bds Ai --> nanti dibuat loop untuk dapat W0 - W7

        outputKi=[]
        outputP=[]
        for i, element in enumerate(Ac_pk):
            # print(f"Indeks {i}: {element}")
            Ai = element
            # print ("====== ITERASI KE -", i+1 , " ======")
            Z = Ai@Y+Bc@Wi
            lmi = cp.vstack([
                cp.hstack([Y, Z.T ,Y, Wi.T]), #baris 1
                cp.hstack([Z, Y, np.zeros([3,3]), np.zeros([3,2])]), #baris 2
                cp.hstack([Y, np.zeros([3,3]), invQts, np.zeros([3,2])]), #baris 3
                cp.hstack([Wi, np.zeros([2,3]), np.zeros([2,3]), invRts]) #baris 4
                ])
            constraints = [lmi>=(0.3*np.eye(11)), Y>>0] # lmi definit positif dgn batasan lebih spesifik agar nilai Y dan Wi tidak nol
            obj = cp.Minimize(0)
            problem = cp.Problem(obj, constraints)
            # print("Solving problem")
            problem.solve(solver=cp.SCS)

            if problem.status == cp.OPTIMAL:
                # print("Optimal value: ", problem.value)
                # print("Y = ", Y.value)
                # print("Wi = " , Wi.value)
                P = np.linalg.inv(Y.value)
                Ki = Wi.value@P
                # print("Ki = ", Ki)
                outputP.append(P)
                outputKi.append(Ki)
            else:
                print("Problem not solved")
                print("Status:", problem.status)
        return outputKi,outputP
    def getLQR() :
        # menambahkan baris
        n = 3  # number of states
        m = 2  # number of inputs
        # Ai=np.random.randint(1,5,(3,3)) #state matrix --> Jumlah Ai nanti ada 8 (sesuai Ac_pk A0 - A7)
        B = np.array([[-0.1,0],[0,0],[0,-0.1]]) # input matrix
        Qts = 0.9*np.diag([0.66, 0.01, 0.33])  # state weight matrix --> JURNAL
        Rts = 0.1*np.diag([0.5, 0.5])  # input weight matrix --> JURNAL
        invRts= np.linalg.inv(Rts)
        invQts= np.linalg.inv(Qts)
        Y = cp.Variable((n, n),symmetric=True)
        Wi = cp.Variable((m, n)) # Solusi bobot untuk kontroler bds Ai --> nanti dibuat loop untuk dapat W0 - W7

        A0 = np.array([[0.43896, -247.10937926, -24.72093793],
                    [0.0, -370.31859311, 94.86714069],
                    [0.0, 23.71703517, 3.37170352]])
        A1 = np.array([[0.43896, -247.10937926, -24.70093793],
                    [0.0, -370.31859311, 94.86714069],
                    [0.0, 23.71703517, 3.37170352]])
        A2 = np.array([[-13.21045, -1.2355469, -0.13355469],
                    [0.0, -0.85659297, 0.2743407],
                    [0.0, 0.11858518, 1.01185852]])
        A3 = np.array([[-13.21045, -1.2355469, -0.11355469],
                    [0.0, -0.85659297, 0.2743407],
                    [0.0, 0.11858518, 1.01185852]])
        A4 = np.array([[0.43896, 247.10937926, 24.70093793],
                    [0.0, -370.31859311, 94.86714069],
                    [0.0, 23.71703517, 3.37170352]])
        A5 = np.array([[0.43896, 247.10937926, 24.72093793],
                    [0.0, -370.31859311, 94.86714069],
                    [0.0, 23.71703517, 3.37170352]])
        A6 = np.array([[-13.21045, 1.2355469, 0.11355469],
                    [0.0, -0.85659297, 0.2743407],
                    [0.0, 0.11858518, 1.01185852]])
        A7 = np.array([[-13.21045, 1.2355469, 0.13355469],
                    [0.0, -0.85659297, 0.2743407],
                    [0.0, 0.11858518, 1.01185852]])
        Ad_vk = [A0, A1, A2, A3, A4, A5, A6, A7]

        outputKi=[]

        for i, element in enumerate(Ad_vk):
            # print(f"Indeks {i}: {element}")
            Ai = element
            # print("A", i, " = ", Ai)
            # print ("====== ITERASI KE -", i+1 , " ======")
            Z = Ai@Y+B@Wi
            lmi = cp.vstack([
                cp.hstack([Y, Z.T ,Y, Wi.T]), #baris 1
                cp.hstack([Z, Y, np.zeros([3,3]), np.zeros([3,2])]), #baris 2
                cp.hstack([Y, np.zeros([3,3]), invQts, np.zeros([3,2])]), #baris 3
                cp.hstack([Wi, np.zeros([2,3]), np.zeros([2,3]), invRts]) #baris 4
                ])
            constraints = [lmi>=(0.3*np.eye(11)), Y>>0] # lmi definit positif dgn batasan lebih spesifik agar nilai Y dan Wi tidak nol
            obj = cp.Minimize(0)
            problem = cp.Problem(obj, constraints)
            print("Solving problem")
            problem.solve(solver=cp.SCS)

            if problem.status == cp.OPTIMAL:
                print("Optimal value: ", problem.value)
                print("Y = ", Y.value)
                print("Wi = " , Wi.value)
                Ki = Wi.value@np.linalg.inv(Y.value)
                print("Ki = ", Ki)
                outputKi.append(Ki)
            else:
                print("Problem not solved")
                print("Status:", problem.status)
        print ("Output Ki : ", outputKi)    

class MPC :
    def __init__(self) -> None:
        pass
    