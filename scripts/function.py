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


    def getLPV(psi_dot, xr_dot, psi_e, Ac_pk):

        nu0_psidot= (psi_dot_max-psi_dot)/(psi_dot_max-psi_dot_min)
        nu1_psidot= 1-nu0_psidot
        nu0_xrdot= (xr_dot_max-xr_dot)/(xr_dot_max-xr_dot_min)
        nu1_xrdot= 1-nu0_xrdot
        nu0_psi= (psi_max-psi_e)/(psi_max-psi_min)
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
        eigenvalues = np.linalg.eigvals(P)
        if np.all(eigenvalues >= 0):
            print("P adalah matriks positif semidefinit.")
        else:
            print("P bukan matriks positif semidefinit.")

        eigenvalues = np.linalg.eigvals(S)
        if np.all(eigenvalues >= 0):
            print("S adalah matriks positif semidefinit.")
        else:
            print("S bukan matriks positif semidefinit.")

        return P, outputKi, S
    def calculate_new_states(Ac_pk, pk, X_k, Bc, U_k, Rc_k, i):
        Ac = Kinematic.getLPV(pk[0], pk[1][i], pk[2], Ac_pk) # Ac is a function of schedulling vector psi_dot, xr_dot[i], psi_e 
        
        # X_k+i+1 = Ac(pk) @ X_k + Bc @ U_k - Bc @ Rc_k[i]
        next_state = Ac@X_k + Bc@U_k - Bc@Rc_k[:,:,i]
        return next_state

    def LPV_MPC(x_k, u_k, r_k, pk, Ac_pk, Bc, P, S):
        X_k = [cp.Variable((n, 1)) for _ in range(N+1)]
        delta_u_k = [cp.Variable((m, 1)) for _ in range(N)]     # Control input at time k
        U_k = [cp.Variable((m, 1)) for _ in range(N)]     # Control input at time k        
        Jk = 0
        for i in range (N):
            # print ("iterasi ke-",i)
            Jk += cp.quad_form(X_k[i], Q_k) + cp.quad_form(delta_u_k[i], R_k)
            if i == 0 :
                constraints = [X_k[i]==x_k]      # Set initial state
                U_k[i-1].value = u_k
            constraints += [U_k[i] == U_k[i-1]+delta_u_k[i]]
            # constraints +=  [X_k[i+1] == Ac @ X_k[i] + Bc @ U_k[i] - Bc @ r_k[:,:,i]]
            constraints += [X_k[i+1] == Kinematic.calculate_new_states(Ac_pk, pk, X_k[i], Bc, U_k[i], r_k, i)]
            constraints += [delta_u_min <= delta_u_k[i], delta_u_k[i] <= delta_u_max]
            constraints += [u_min <= U_k[i], U_k[i] <= u_max]
        # Jk += X_k[N].T @ P @ X_k[N]
        # constraints += [X_k[N].T @ S @ X_k[N]<=1]
        Jk += cp.quad_form(X_k[N], P)
        constraints += [cp.quad_form(X_k[N], S)<=1]
        # Define and solve the optimization problem

        objective = cp.Minimize(Jk)
        problem = cp.Problem(objective, constraints)
        # problem.solve(solver=cp.GUROBI, verbose=True)  
        problem.solve(solver=cp.SCS, verbose=False) # yang bisa ECOS,SCS
        if problem.status == cp.OPTIMAL:
            print("Jk_optimized = ", problem.value)
            Xk_optimized = np.zeros([N+1,3,1])
            Delta_U_optimized = np.zeros([N,2,1])
            U_k_optimized = np.zeros([N,2,1])
            for  j in range(N+1):
                Xk_optimized[j]=X_k[j].value
                if j < N:
                    Delta_U_optimized[j]=delta_u_k[j].value
                    U_k_optimized[j]=U_k[j].value

            u_opt = U_k_optimized[0]
            x_opt = Xk_optimized[1]
            print("next_x_opt : ", x_opt[1])
        else:
            print("Problem not solved")
            print("Status:", problem.status)
            # Agar program tidak eror dan loop terus berjalan, 
            # maka nilai u yang di return adalah nilai sebelumnya
            x_opt = 0
            u_opt = u_k + delta_u_max/2
        
        # print("Xk_optimized", Xk_optimized)
        # print("Delta_U_optimized", Delta_U_optimized)
        # print("U_k_optimized", U_k_optimized)
            
        return x_opt, u_opt
        #===========================================================
    
    def MPC2(x_k, u_k, r_k, Ac_pk, Ac_0, Bc, P, S): 
        X_k = [cp.Variable((n, 1)) for _ in range(N+1)]
        delta_u_k = [cp.Variable((m, 1)) for _ in range(N)]     # Control input at time k
        U_k = [cp.Variable((m, 1)) for _ in range(N)]     # Control input at time k        
        Jk = 0
        for i in range (N):
            # print ("iterasi ke-",i)
            # print ("Curvature X", (X_k[i].T @ Q_k @ X_k[i]).curvature)
            # print ("Curvature X", cp.quad_form(X_k[i], Q_k).curvature)
            # Jk += X_k[i].T @ Q_k @ X_k[i] + delta_u_k[i].T @ R_k @ delta_u_k[i]
            Jk += cp.quad_form(X_k[i], Q_k) + cp.quad_form(delta_u_k[i], R_k)
            if i == 0 :
                constraints = [X_k[i]==x_k]      # Set initial state
                U_k[i-1].value = u_k
                Ac = Ac_0
            else :
                xr_dot = r_k[0,0,i]
                psi_dot = U_k[i-1][1]
                psi_e = X_k[i][2]
                constraints += [Ac == Kinematic.getLPV(psi_dot, xr_dot, psi_e, Ac_pk)]
            constraints += [U_k[i] == U_k[i-1]+delta_u_k[i]]
            constraints +=  [X_k[i+1] == Ac @ X_k[i] + Bc @ U_k[i] - Bc @ r_k[:,:,i]]
            constraints += [delta_u_min <= delta_u_k[i], delta_u_k[i] <= delta_u_max]
            constraints += [u_min <= U_k[i], U_k[i] <= u_max]
        # Jk += X_k[N].T @ P @ X_k[N]
        # constraints += [X_k[N].T @ S @ X_k[N]<=1]
        Jk += cp.quad_form(X_k[N], P)
        constraints += [cp.quad_form(X_k[N], S)<=1]
        # Define and solve the optimization problem

        objective = cp.Minimize(Jk)
        problem = cp.Problem(objective, constraints)
        # problem.solve(solver=cp.GUROBI, verbose=True)  
        problem.solve(solver=cp.SCS, verbose=False) # yang bisa ECOS,SCS
        if problem.status == cp.OPTIMAL:
            print("Jk_optimized = ", problem.value)
            Xk_optimized = np.zeros([N+1,3,1])
            Delta_U_optimized = np.zeros([N,2,1])
            U_k_optimized = np.zeros([N,2,1])
            for  j in range(N+1):
                Xk_optimized[j]=X_k[j].value
                if j < N:
                    Delta_U_optimized[j]=delta_u_k[j].value
                    U_k_optimized[j]=U_k[j].value

            u_opt = U_k_optimized[0]
            x_opt = Xk_optimized[1]
            print("next_x_opt : ", x_opt[1])
        else:
            print("Problem not solved")
            print("Status:", problem.status)
            # Agar program tidak eror dan loop terus berjalan, 
            # maka nilai u yang di return adalah nilai sebelumnya
            x_opt = 0
            u_opt = u_k + delta_u_max/2
        
        # print("Xk_optimized", Xk_optimized)
        # print("Delta_U_optimized", Delta_U_optimized)
        # print("U_k_optimized", U_k_optimized)
            
        return x_opt, u_opt
        #===========================================================
        


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