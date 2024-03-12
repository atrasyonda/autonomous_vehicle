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

class Kinematic:
    def __init__(self) -> None:
        pass
    def getLPV(car:state):
        xr_dot = car.xr_dot
        psi_dot= car.psi_dot
        psi=car.psi

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

        return Ac,Ac_pk,Bc
    def getMPCSet(Ac_pk,Bc):
        invQts= np.linalg.inv(Q_k)
        invRts= np.linalg.inv(R_k)
        Y = cp.Variable((n, n),symmetric=True)
        Wi = cp.Variable((m, n)) # Solusi bobot untuk kontroler bds Ai 

        outputKi=[]
        outputP=[]
        for i, element in enumerate(Ac_pk):
            # print(f"Indeks {i}: {element}")
            Ai = element
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
            problem.solve(solver=cp.SCS)

            if problem.status == cp.OPTIMAL:
                P = np.linalg.inv(Y.value)
                Ki = Wi.value@P
                outputP.append(P)
                outputKi.append(Ki)
            else:
                print("Problem not solved")
                print("Status:", problem.status)
        return outputKi,outputP

class Dynamic:
    def __init__(self) -> None:
        pass
    def getModel():
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

        Bd= np.array([
            [0, Td],
            [Td*Caf/m, 0],
            [Td*Caf*lf/I, 0]
        ])
        return Ad_vk, Bd
    
    def getLPV(car:state, Ad_vk):
        x_dot = car.x_dot
        y_dot = car.y_dot
        delta = car.delta #sudut steering
    
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
        Ad=0
        for i in range(2**n):
            Ad+=(Ad_vk[i]*miu_vk[i])

        return miu_vk, Ad
    
    def getLQR(Ad_vk,Bd): 
        invQts= np.linalg.inv(Q_d)
        invRts= np.linalg.inv(R_d)
        Y = cp.Variable((n, n),symmetric=True)
        Wi = cp.Variable((m, n)) # Solusi bobot untuk kontroler bds Ai --> nanti dibuat loop untuk dapat W0 - W7
        outputKi=[]
        for i, element in enumerate(Ad_vk):
            # print(f"Indeks {i}: {element}")
            Ai = element
            Z = Ai@Y+Bd@Wi
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
        return outputKi   
    def evaluateLQR(miu_vk,outputKi):
        K_vk=0
        for i in range(2**n):
            K_vk +=(miu_vk[i]*outputKi[i])
        return K_vk