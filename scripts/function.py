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
        Ac_pk = np.array([A0, A1, A2, A3, A4, A5, A6, A7]) # matrix 8x3x3
        Bc= np.array([
            [-Tc,0],
            [0,0],
            [0,-Tc]
        ]) # matrix 3x2
        return Ac_pk, Bc


    def getLPV(car:state, Ac_pk):
        psi_dot= car.psi_dot
        xr_dot = car.x_dot_ref
        psi=car.psi_e

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
            
        # print("Output P", P)
        # print("Output Ki", outputKi)
        S = np.array([
            [0.465, 0, 0],
            [0, 23.813, 76.596],
            [0, 76.596, 257.251]
        ])  # INI MATRIX S DARI JURNAL REFERENSI
        # print ("Output S:" , S)
        print ("S eigenvalues", np.linalg.eigvals(S)) # check if S is positive definite


        return P, outputKi, S
    def MPC(x_k, u_k, r_k, Ac, Bc, P, Ki, S): # 
        """=====================================================
        A_aug = [A , B] --> dimension (5x5)
                [O , I]
        state ---> [x_k+1] = [x k]   --> dimension (5x1)
                    [u_k]    [u_k-1]
        B_aug = [B] --> dimension (5x2)
                [I]
        input ---> [delta_u_k] --> dimension (2x1)

        A_aug=np.concatenate((Ac,Bc),axis=1)
        temp1=np.zeros((np.size(Bc,1),np.size(Ac,1)))
        temp2=np.identity(np.size(Bc,1))
        temp=np.concatenate((temp1,temp2),axis=1)
        A_aug=np.concatenate((A_aug,temp),axis=0)
        # print("Ac", Ac)
        # print("Bc", Bc)
        # print("A_aug: ", A_aug)
        B_aug=np.concatenate((Bc,np.identity(np.size(Bc,1))),axis=0)
        # print("B_aug: ", B_aug.shape)
        ========================================================
        """
        # ====== DARI CHAT GPT ======== 
        X_k = [cp.Variable((n, 1)) for _ in range(N+1)]
        delta_u_k = [cp.Variable((m, 1)) for _ in range(N)]     # Control input at time k
        U_k = [cp.Variable((m, 1)) for _ in range(N)]     # Control input at time k        
        Jk = 0
        for i in range (N):
            # print("iterasi ke-",i)
            Jk += cp.quad_form(X_k[i], Q_k) + cp.quad_form(delta_u_k[i], R_k)
            if i == 0 :
                constraints = [X_k[i]==x_k]      # Set initial state
                U_k[i-1].value = u_k
            constraints += [U_k[i] == U_k[i-1]+delta_u_k[i]]
            constraints +=  [X_k[i+1] == Ac @ X_k[i] + Bc @ delta_u_k[i] - Bc @ r_k]
            constraints += [delta_u_min <= delta_u_k[i], delta_u_k[i] <= delta_u_max]
            constraints += [u_min <= U_k[i], U_k[i] <= u_max]
        Jk += cp.quad_form(X_k[N], P)
        constraints += [cp.quad_form(X_k[N], S)<=1]
        # Define and solve the optimization problem
        objective = cp.Minimize(Jk)
        problem = cp.Problem(objective, constraints)
        problem.solve(solver=cp.GUROBI, verbose=True)
        if problem.status == cp.OPTIMAL:
            print("Jk_optimized = ", problem.value)
            print ("State Optimized")
            for  j in range(len(X_k)):
                print(X_k[j].value)
            print ("Input")
            for  j in range(len(X_k)):
                print(X_k[j].value)
        else:
            print("Problem not solved")
            print("Status:", problem.status)


        
    def MPC2(x_k, u_k, r_k, Ac, Bc, P, Ki, S): 
        #===== CALCULATE ESTIMATE STATE DURING HORIZON PERIOD ======
        X_k = cp.Variable((n,N+1))   # State at time k
        delta_u_k = cp.Variable((m,N))     # Control input at time k
        U_k = cp.Parameter((m,N))
        
        Jk = 0
        for i in range (N):
            print("iterasi ke-",i)
            Jk += cp.quad_form(X_k[:,i], Q_k) + cp.quad_form(delta_u_k[:,i], R_k)
            if i == 0 :
                constraints = [X_k[:,i]==x_k]      # Set initial state
                constraints += [U_k[:,i-1] == u_k] # Set previous input
            constraints += [u_k[:,i] == u_k[:,i-1]+delta_u_k[:,i]]
            constraints +=  [X_k[:,i+1] == Ac @ X_k[:,i] + Bc @ delta_u_k[:,i] - Bc @ r_k]
            constraints += [delta_u_min <= delta_u_k[:,i], delta_u_k[:,i] <= delta_u_max]
            constraints += [u_min <= u_k[:,i], u_k[:,i] <= u_max]
        Jk += cp.quad_form(X_k[:,N], P)
        constraints += [cp.quad_form(X_k[N], S)<=1]
        # Define and solve the optimization problem
        objective = cp.Minimize(Jk)
        problem = cp.Problem(objective, constraints)
        problem.solve(solver=cp.GUROBI, verbose=False)
        if problem.status == cp.OPTIMAL:
            print("Jk_optimized ke-",i," = ", problem.value)
            print("X_k", X_k.value)
            print("Delta_u", delta_u_k.value)
        else:
            print("Problem not solved")
            print("Status:", problem.status)


            # if i == 0 :
            #     X_k[i]= x_k
            #     U_k[i]= u_k
            # else :
            #     U_k[i]= U_k[i-1]+delta_u_opt[i-1]
            #     X_k[i]= Ac@X_k[i-1]+Bc@U_k[i]-Bc@rc_k
            
            # Jk = sum([cp.quad_form(X_k[i], Q_k) + cp.quad_form(delta_u[i], R_k)])
            # objective = cp.Minimize(Jk)
            # constraints = [
            #     delta_u[i]>=delta_u_min, delta_u[i]<=delta_u_max
            # ]
            # problem = cp.Problem(objective, constraints)
            # problem.solve(solver=cp.GUROBI, verbose=False)
            # if problem.status == cp.OPTIMAL:
            #     print("Jk_optimized= ", problem.value)
            #     print("delta_u_optimized = ", delta_u[i].value)
            #     delta_u_opt[i] = delta_u[i].value
            # else:
            #     print("Problem not solved")
            #     print("Status:", problem.status)
        #===========================================================

        #===========================================================
    def MPC3(x_k, u_k, rc_k, Ac, Bc, P, Ki, S): # 
        """=====================================================
        A_aug = [A , B] --> dimension (5x5)
                [O , I]
        state ---> [x_k+1] = [x k]   --> dimension (5x1)
                    [u_k]    [u_k-1]
        B1_aug = [B] --> dimension (5x2)
                [I]
        input ---> [delta_u_k] --> dimension (2x1)
        B2_aug = [B] --> dimension (5x2)
                 [O]   
        reference rc ---> [vd cos psi , omega_d] --> dimension (2x1)              
        ========================================================
        """
        A_aug=np.concatenate((Ac,Bc),axis=1)
        temp1=np.zeros((np.size(Bc,1),np.size(Ac,1)))
        temp2=np.identity(np.size(Bc,1))
        temp=np.concatenate((temp1,temp2),axis=1)
        A_aug=np.concatenate((A_aug,temp),axis=0)
        # print("Ac", Ac)
        # print("Bc", Bc)
        # print("A_aug: ", A_aug)
        B_aug=np.concatenate((Bc,np.identity(np.size(Bc,1))),axis=0)
        # print("B_aug: ", B_aug.shape)
        G = np.zeros([1,2])
        H = np.zeros([2,1])
        I = G@H
        print("===================")
        print(G)
        print(H)
        print("===================")
        print(I)


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
        delta = car.delta #sudut steering
        x_dot = car.x_dot
        y_dot = car.y_dot
    
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