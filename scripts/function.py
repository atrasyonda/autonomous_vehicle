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
    def getModel():
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
        Ac_pk = np.array([A0, A1, A2, A3, A4, A5, A6, A7])
        Bc= np.array([
            [-Tc,0],
            [0,0],
            [0,-Tc]
        ])
        return Ac_pk, Bc


    def getLPV(car:state, Ac_pk):
        xr_dot = car.xr_dot
        psi_dot= car.psi_dot
        psi=car.psi

        nu0_psidot= (psi_dot_max-psi_dot)/(psi_dot_max-psi_dot_min)
        nu1_psidot= 1-nu0_psidot
        nu0_xrdot= (xr_dot_max-xr_dot)/(xr_dot_max-xr_dot_min)
        nu1_xrdot= 1-nu0_xrdot
        nu0_psi= (psi_max-psi)/(psi_max-psi_min)
        nu1_psi= 1-nu0_psi
        
        miu_pk = np.array([
            [nu0_psidot*nu0_xrdot*nu0_psi],
            [nu0_psidot*nu0_xrdot*nu1_psi],
            [nu0_psidot*nu1_xrdot*nu0_psi],
            [nu0_psidot*nu1_xrdot*nu1_psi],
            [nu1_psidot*nu0_xrdot*nu0_psi],
            [nu1_psidot*nu0_xrdot*nu1_psi],
            [nu1_psidot*nu1_xrdot*nu0_psi],
            [nu1_psidot*nu1_xrdot*nu1_psi],
        ])
        Ac=0
        for i in range(2**n):
            Ac+=(Ac_pk[i]*miu_pk[i])
        return Ac
    
    def getMPCSet(Ac_pk,Bc):
        invQts= np.linalg.inv(Q_k)
        invRts= np.linalg.inv(R_k)
        Y = cp.Variable((n, n),symmetric=True)
        Wi = cp.Variable((m, n)) # Solusi bobot untuk kontroler bds Ai      
        outputKi= np.zeros([8,2,3])
        for i in range (2**n):
            # print(f"Indeks {i}: {element}")
            Ai = Ac_pk[i]
            if i == 0 :
                lmi = cp.vstack([
                    cp.hstack([Y, (Ai@Y+Bc@Wi).T ,Y, Wi.T]), #baris 1
                    cp.hstack([(Ai@Y+Bc@Wi), Y, np.zeros([3,3]), np.zeros([3,2])]), #baris 2
                    cp.hstack([Y, np.zeros([3,3]), invQts, np.zeros([3,2])]), #baris 3
                    cp.hstack([Wi, np.zeros([2,3]), np.zeros([2,3]), invRts]) #baris 4
                    ])
                constraints = [lmi>=(0.3*np.eye(11)), Y>>0] # lmi definit positif dgn batasan lebih spesifik agar nilai Y dan Wi tidak nol
                obj = cp.Minimize(0)
                problem = cp.Problem(obj, constraints)
                problem.solve(solver=cp.SCS)
                if problem.status == cp.OPTIMAL:
                    y_opt = Y.value
                    P = np.linalg.inv(y_opt)
                    Ki = Wi.value@P
                    outputKi[i]=Ki
                else:
                    print("Problem not solved")
                    print("Status:", problem.status)
            else :
                lmi = cp.vstack([
                    cp.hstack([y_opt, (Ai@y_opt+Bc@Wi).T ,y_opt, Wi.T]), #baris 1
                    cp.hstack([(Ai@y_opt+Bc@Wi), y_opt, np.zeros([3,3]), np.zeros([3,2])]), #baris 2
                    cp.hstack([y_opt, np.zeros([3,3]), invQts, np.zeros([3,2])]), #baris 3
                    cp.hstack([Wi, np.zeros([2,3]), np.zeros([2,3]), invRts]) #baris 4
                    ])
                constraints = [lmi>=(0.3*np.eye(11))] # lmi definit positif dgn batasan lebih spesifik agar nilai Y dan Wi tidak nol
                obj = cp.Minimize(0)
                problem = cp.Problem(obj, constraints)
                problem.solve(solver=cp.SCS)
                if problem.status == cp.OPTIMAL:
                    Ki = Wi.value@P
                    outputKi[i]=Ki
                else:
                    print("Problem not solved")
                    print("Status:", problem.status)
                    outputKi[i]=0
        print("Output P", P)
        print("Output Ki", outputKi)
        #================================================================================================
        Z = cp.Variable((n, n), symmetric=True)
        u_bar= np.array([[1.4], [20]]) # matrix 2x1
        u_bar_squared= u_bar@u_bar.T

        S= np.zeros([3,3])
        lmi2=[]

        for i in range (2**n):
            Ai = Ac_pk[i]
            Ki = outputKi[i]
            lmi_prob = cp.vstack([
                cp.hstack([-Z, Z@(Ai+Bc@Ki).T]), #baris 1
                cp.hstack([(Ai+Bc@Ki)@Z, -Z]), #baris 2
                ])
            lmi2.append(lmi_prob)

        # print ("lmi2 ke 1", lmi2[0])
        constraints2 = [lmi2[0]<<0, 
                        lmi2[1]<<0,
                        lmi2[2]<<0,
                        lmi2[3]<<0,
                        lmi2[4]<<0,
                        lmi2[5]<<0,
                        lmi2[6]<<0,
                        lmi2[7]<<0,
                        Ki@Z@Ki.T-u_bar_squared<<0] 
        obj2 = cp.Maximize(0)
        problem2 = cp.Problem(obj2, constraints2)
        problem2.solve(solver=cp.SCS)
        if problem2.status == cp.OPTIMAL:
            S=np.linalg.inv(Z.value)
        else:
            print("Problem not solved")
            print("Status:", problem2.status)
        print ("Output S:" , S)

        return P, outputKi, S

class Dynamic:
    def __init__(self) -> None:
        pass
    def getModel():
        A0=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_min**2 + miu*mass*g)/ (mass*x_dot_min))*Td, ((Caf*np.sin(delta_min))/(mass*x_dot_min))*Td, (Caf*lf*np.sin(delta_min)/(mass*x_dot_min)+y_dot_min)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_min))/(mass*x_dot_min))*Td, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(mass*x_dot_min)-x_dot_min)*Td],
            [0, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(I*x_dot_min))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_min)-Car*lr**2)/(I*x_dot_min))*Td]
        ])
        A1=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_min**2 + miu*mass*g)/ (mass*x_dot_min))*Td, ((Caf*np.sin(delta_min))/(mass*x_dot_min))*Td, (Caf*lf*np.sin(delta_min)/(mass*x_dot_min)+y_dot_max)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_min))/(mass*x_dot_min))*Td, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(mass*x_dot_min)-x_dot_min)*Td],
            [0, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(I*x_dot_min))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_min)-Car*lr**2)/(I*x_dot_min))*Td]
        ])
        A2=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_max**2 + miu*mass*g)/ (mass*x_dot_max))*Td, ((Caf*np.sin(delta_min))/(mass*x_dot_max))*Td, (Caf*lf*np.sin(delta_min)/(mass*x_dot_max)+y_dot_min)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_min))/(mass*x_dot_max))*Td, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(mass*x_dot_max)-x_dot_max)*Td],
            [0, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(I*x_dot_max))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_min)-Car*lr**2)/(I*x_dot_max))*Td]
        ])
        A3=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_max**2 + miu*mass*g)/ (mass*x_dot_max))*Td, ((Caf*np.sin(delta_min))/(mass*x_dot_max))*Td, (Caf*lf*np.sin(delta_min)/(mass*x_dot_max)+y_dot_max)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_min))/(mass*x_dot_max))*Td, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(mass*x_dot_max)-x_dot_max)*Td],
            [0, (-(Caf*lf*np.cos(delta_min)-Car*lr)/(I*x_dot_max))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_min)-Car*lr**2)/(I*x_dot_max))*Td]
        ])
        A4=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_min**2 + miu*mass*g)/ (mass*x_dot_min))*Td, ((Caf*np.sin(delta_max))/(mass*x_dot_min))*Td, (Caf*lf*np.sin(delta_max)/(mass*x_dot_min)+y_dot_min)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_max))/(mass*x_dot_min))*Td, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(mass*x_dot_min)-x_dot_min)*Td],
            [0, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(I*x_dot_min))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_max)-Car*lr**2)/(I*x_dot_min))*Td]
        ])
        A5=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_min**2 + miu*mass*g)/ (mass*x_dot_min))*Td, ((Caf*np.sin(delta_max))/(mass*x_dot_min))*Td, (Caf*lf*np.sin(delta_max)/(mass*x_dot_min)+y_dot_max)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_max))/(mass*x_dot_min))*Td, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(mass*x_dot_min)-x_dot_min)*Td],
            [0, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(I*x_dot_min))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_max)-Car*lr**2)/(I*x_dot_min))*Td]
        ])
        A6=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_max**2 + miu*mass*g)/ (mass*x_dot_max))*Td, ((Caf*np.sin(delta_max))/(mass*x_dot_max))*Td, (Caf*lf*np.sin(delta_max)/(mass*x_dot_max)+y_dot_min)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_max))/(mass*x_dot_max))*Td, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(mass*x_dot_max)-x_dot_max)*Td],
            [0, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(I*x_dot_max))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_max)-Car*lr**2)/(I*x_dot_max))*Td]
        ])
        A7=np.array([
            [1+(-(0.5*rho*Cd*Af*x_dot_max**2 + miu*mass*g)/ (mass*x_dot_max))*Td, ((Caf*np.sin(delta_max))/(mass*x_dot_max))*Td, (Caf*lf*np.sin(delta_max)/(mass*x_dot_max)+y_dot_max)*Td],
            [0, 1+(-(Car+Caf*np.cos(delta_max))/(mass*x_dot_max))*Td, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(mass*x_dot_max)-x_dot_max)*Td],
            [0, (-(Caf*lf*np.cos(delta_max)-Car*lr)/(I*x_dot_max))*Td, 1+(-(Caf*(lf**2)*np.cos(delta_max)-Car*lr**2)/(I*x_dot_max))*Td]
        ])
        Ad_vk=np.array([A0,A1,A2,A3,A4,A5,A6,A7])

        Bd= np.array([
            [0, Td],
            [Td*Caf/mass, 0],
            [Td*Caf*lf/I, 0]
        ])
        # print("Ad_vk: ", Ad_vk)
        # print("Bd: ", Bd)
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

        miu_vk = np.array([
            [nu0_delta*nu0_xdot*nu0_ydot],
            [nu0_delta*nu0_xdot*nu1_ydot],
            [nu0_delta*nu1_xdot*nu0_ydot],
            [nu0_delta*nu1_xdot*nu1_ydot],
            [nu1_delta*nu0_xdot*nu0_ydot],
            [nu1_delta*nu0_xdot*nu1_ydot],
            [nu1_delta*nu1_xdot*nu0_ydot],
            [nu1_delta*nu1_xdot*nu1_ydot],
        ])
        Ad=np.zeros([3,3])

        for i in range(2**n):
            Ad+=(miu_vk[i]*Ad_vk[i])
        # print ("Ad : ", Ad)
        return miu_vk, Ad
    
    def getLQR(Ad_vk,Bd): 
        invQts= np.linalg.inv(Q_d)
        invRts= np.linalg.inv(R_d)
        Y = cp.Variable((n, n),symmetric=True)
        Wi = cp.Variable((m, n)) # Solusi bobot untuk kontroler bds Ai --> nanti dibuat loop untuk dapat W0 - W7

        Ki = np.zeros([2,3])
        outputKi=np.zeros([8,2,3])

        for i, element in enumerate(Ad_vk):
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
            problem.solve(solver=cp.SCS)

            if problem.status == cp.OPTIMAL:
                Ki = Wi.value@np.linalg.inv(Y.value)
                outputKi[i]=Ki
            else:
                print("Problem not solved")
                print("Status:", problem.status)
        # print ("Output Ki : ", outputKi)
        return outputKi 
      
    def evaluateLQR(miu_vk,outputKi):
        K_vk=0
        for i in range(2**n):
            K_vk +=(miu_vk[i]*outputKi[i])
        return K_vk