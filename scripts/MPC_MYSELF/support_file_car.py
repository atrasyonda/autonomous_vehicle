import numpy as np
import matplotlib as plt
import random
# from constants import *

class LPV:
    def __init__(self) -> None:
        pass
    def getKinematic(self):
        Tc=0.1
        psi_dot_min= -1.42
        psi_dot_max= 1.42
        xr_dot_min= 0.1
        xr_dot_max= 20
        psi_min= -0.05
        psi_max= 0.05
        psi_dot = round(random.uniform(psi_dot_min, psi_dot_max),2)
        xr_dot = round(random.uniform(xr_dot_min, xr_dot_max),2)
        psi = round(random.uniform(psi_min, psi_max),2)

        nu0_psidot= (psi_dot_max-psi_dot)/(psi_dot_max-psi_dot_min)
        nu1_psidot= 1-nu0_psidot
        nu0_xrdot= (xr_dot_max-xr_dot)/(xr_dot_max-xr_dot_min)
        nu1_xrdot= 1-nu0_xrdot
        nu0_psi= (psi_max-psi)/(psi_max-psi_min)
        nu1_psi= 1-nu0_psi

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

        miu = [
            [nu0_psidot*nu0_xrdot*nu0_psi],
            [nu0_psidot*nu0_xrdot*nu1_psi],
            [nu0_psidot*nu1_xrdot*nu0_psi],
            [nu0_psidot*nu1_xrdot*nu1_psi],
            [nu1_psidot*nu0_xrdot*nu0_psi],
            [nu1_psidot*nu0_xrdot*nu1_psi],
            [nu1_psidot*nu1_xrdot*nu0_psi],
            [nu1_psidot*nu1_xrdot*nu1_psi],
        ]
        Ac=0
        for i in range(8):
            Ac+=(Ac_pk[i]*miu[i])


        return Ac

    def getDynamic(self):
        #constant
        m = 5
        lf = 0.1
        lr = 0.1
        I = 20
        Caf = 19000
        Car = 33000
        Td = 0.005

        rho = 12
        Cd = 20
        Af = 30
        miu = 0.5
        g = 9.8
        # schedulling vector variable
        delta_min = -0.25
        delta_max = 0.25
        x_dot_min = 0.1
        x_dot_max = 20
        y_dot_min = -1
        y_dot_max = 1


        delta=round(random.uniform(delta_min, delta_max),2)
        x_dot=round(random.uniform(x_dot_min, x_dot_max),2)
        y_dot=round(random.uniform(y_dot_min, y_dot_max),2)

        print("======================")
        print(delta)
        print(x_dot)
        print(y_dot)
        print("======================")

        nu0_delta= (delta_max-delta)/(delta_max-delta_min)
        nu1_delta= 1-nu0_delta
        nu0_xdot= (x_dot_max-x_dot)/(x_dot_max-x_dot_min)
        nu1_xdot= 1-nu0_xdot
        nu0_ydot= (y_dot_max-y_dot)/(y_dot_max-y_dot_min)
        nu1_ydot= 1-nu0_ydot

        miu_pk = [
            [nu0_delta*nu0_xdot*nu0_ydot],
            [nu0_delta*nu0_xdot*nu1_ydot],
            [nu0_delta*nu1_xdot*nu0_ydot],
            [nu0_delta*nu1_xdot*nu1_ydot],
            [nu1_delta*nu0_xdot*nu0_ydot],
            [nu1_delta*nu0_xdot*nu1_ydot],
            [nu1_delta*nu1_xdot*nu0_ydot],
            [nu1_delta*nu1_xdot*nu1_ydot],
        ]
        
        delta_min
        delta_max
        x_dot_min
        x_dot_max
        y_dot_min
        y_dot_max

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
        # Ad=np.array([
        #     [1+A11*Td, A12*Td, A13*Td],
        #     [0, 1+A22*Td, A23*Td],
        #     [0, A32*Td, 1+A33*Td]
        # ])
       
        Ad_pk=[A0,A1,A2,A3,A4,A5,A6,A7]
        Ad=0
        for i in range(8):
            Ad+=(Ad_pk[i]*miu_pk[i])
        
        # print (Ad)

        return Ad

class kinematic:
    def __init__(self) :
        # variabel untuk kinematic control
        Tc=0.1
        psi = 1
        psi_dot = 1
        xr_dot=3
        self.constants=[Tc,psi,psi_dot,xr_dot]
        return None
    def state_space(self):
        Tc=self.constants[0]
        psi=self.constants[1]
        psi_dot=self.constants[2]
        xr_dot=self.constants[3]

        A=np.array([
            [1, psi_dot*Tc, 0],
            [-psi_dot*Tc, 1, xr_dot*np.sin(psi)*Tc/psi],
            [0, 0, 1]
        ])
        B=np.array([
            [-Tc, 0],
            [0, 0],
            [0, -Tc]
        ])
        return A,B
class dynamic:
    def __init__(self) :
        # === Vehicle Constant untuk Dynamic Control==
        m = 5
        lf = 0.1
        lr = 0.1
        I = 20
        # Referensi course MPC UDEMY
        Caf = 19000
        Car = 33000
        # Time Sampling --> jurnal LPV-MPC
        Td = 0.005
        self.constants=[m, lf, lr, I, Caf, Car, Td]
        return None
    
    def state_space(self):
        m=self.constants[0]
        lf=self.constants[1]
        lr=self.constants[2]
        I=self.constants[3]
        Caf=self.constants[4]
        Car=self.constants[5]
        Td=self.constants[6]
        
        delta=1
        x_dot=1
        y_dot=2
        rho = 12
        Cd = 20
        Af = 30
        miu = 0.5
        g = 9.8


        A11= -(0.5*rho*Cd*Af*x_dot**2 + miu*m*g)/ (m*x_dot)
        A12= (Caf*np.sin(delta))/(m*x_dot)
        A13= Caf*lf*np.sin(delta)/(m*x_dot)+y_dot
        A22= -(Car+Caf*np.cos(delta))/(m*x_dot)
        A23= -(Caf*lf*np.cos(delta)-Car*lr)/(m*x_dot)-x_dot
        A32= -(Caf*lf*np.cos(delta)-Car*lr)/(I*x_dot)
        A33= -(Caf*(lf**2)*np.cos(delta)-Car*lr**2)/(I*x_dot)

        A=np.array([
            [1+A11*Td, A12*Td, A13*Td],
            [0, 1+A22*Td, A23*Td],
            [0, A32*Td, 1+A33*Td]
        ])

        B=np.array(
            [[0, 1],
             [Caf/m, 0],
             [Caf*lf/I, 0]
        ])*Td

        return A,B  

