#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
from function import Kinematic, Dynamic
from constants import *

# Create list to store the data
iteration=[]
car_pos_x = [] 
car_pos_y= [] 
car_psi = []
car_delta = []
car_x_dot = []
car_psi_dot = []
ref_pos_x = []
ref_pos_y = []
ref_psi = []
ref_x_dot = []
ref_psi_dot = []

def path_generator():
   # Plot the reference trajectory
    trajectory = 3
    t=np.arange(0,10+Tc,Tc) # duration of the entire manoeuvre
    lane_width=7 # [m]
    r=8
    f=0.015
    x_dot = 10
    
    # Define the x length, depends on the car's longitudinal velocity
    x=np.linspace(0,x_dot*t[-1],num=len(t))
    
    if trajectory==1:
        y=-9*np.ones(len(t))
    elif trajectory==2:
        y=9*np.tanh(t-t[-1]/2)
    elif trajectory==3:
        aaa=-28/100**2
        aaa=aaa/1.1
        if aaa<0:
            bbb=14
        else:
            bbb=-14
        y_1=aaa*(x+lane_width-100)**2+bbb
        y_2=2*r*np.sin(2*np.pi*f*x)
        y=(y_1+y_2)/2
        # y=(y_2)/2
    
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

    Vd = dx/Tc
    Wd = dpsi/Tc

    # Inisialisasi buffer dengan N elemen pertama dari data (N = horizon prediction)
    buffer_size = N
    Xr_dot = [Vd[i:i+buffer_size] for i in range(len(Vd) - buffer_size + 1)]
    Psi_dot = [Wd[i:i+buffer_size] for i in range(len(Wd) - buffer_size + 1)]

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
P, Ki_k, S= Kinematic.getMPCSet(Ac_pk,Bc) 
# set initial variabel

Ad_vk, Bd = Dynamic.getModel()
Ki_d = Dynamic.getLQR(Ad_vk, Bd)
# print ("Dynamic LMI Gain : ", Ki_d)

def kinematic_control (data):
    # Construct Vector of Schedulling Variables
    pk = [data.psi_dot, data.x_dot_ref, data.psi]
    # Construct the State-Space model
    X_k = np.array([[data.x], [data.y], [data.psi]])  # get current error state 
    U_k = np.array([[data.x_dot], [data.psi_dot]]) # get previous control signal
    # Construct reference signal for N horizon prediction
    xr_dot_psi_e = [x * np.cos(data.psi) for x in data.x_dot_ref]
    Rc_k = np.array([[xr_dot_psi_e], [data.psi_dot_ref]])
    next_x_opt, u_opt = Kinematic.LPV_MPC(X_k, U_k, Rc_k, pk, Ac_pk, Bc, P, S)
    # ======= Calculate next state ========
    control_signal = u_opt
    # control_signal = U_k
    next_state = Kinematic.calculate_new_states(Ac_pk, pk, X_k, Bc, control_signal, Rc_k, 0)
    print("=====================================")
    print("next_state : ", next_state)
    print("control_signal : ", control_signal)
    return next_state, control_signal

def dynamic_control (data):
    X_d = np.array([[data.x_dot], [data.y_dot],[data.psi_dot]])
    X_e = setpoint_dynamic - X_d

    vk = [data.delta, data.x_dot, data.y_dot] 
    Ad, K_vk= Dynamic.LPV_LQR(vk, Ad_vk, Ki_d)
    Ad_cl = Ad - Bd@K_vk
    
    U_d = K_vk @ X_e

    print("=====================================")
    print("Error State : ", X_e)
    print("Gain K_vk : ", K_vk)
    print("=====================================")

    # delta_car = np.arctan(U_k[1,0]*(lf+lr)/U_k[0,0]) + U_d[0,0]
    # a = U_d[1,0]
    # x_dot_car = U_k[0,0] + a*Td
    # U_cd = [delta_car, x_dot_car]

    # U_cd = U_d + U_k
    next_d_state = Ad_cl @ X_d + Bd @ U_d



    # print("=====================================")
    # print("LQR Gain : ", K_vk)
    print("U_d : ", U_d)
    # print("=====================================")

    return next_d_state, U_d

def calculate_new_states():
    # ======= Calculate next state ========
    # input delta steering and Vx

    # Fyf = (Caf*(delta_steer - y_dot/x_dot - lf*psi_dot/x_dot))
    # Fyr = (Car*(-y_dot/x_dot + lr*psi_dot/x_dot))
    # Vx_next = a - Fyf*np.sin(delta_steer)/m - (0.5*Cd*rho*Af*x_dot**2 + miu*m*g)/m + psi_dot*y_dot
    # Vy_next = Fyf*np.cos(delta_steer)/m + Fyr/m - psi_dot*x_dot
    # Psi_dot_next = (Fyf*lf*np.cos(delta_steer) - Fyr*lr)/I 

    # Xe_next = psi_dot*Y_k + xr_dot[i][0]*np.cos(Psi_k) - x_dot
    # Ye_next = -psi_dot*X_k + xr_dot[i][0]*np.sin(Psi_k)
    # Psi_next = psi_dot - Psi_k
    return next_k_state


if __name__=='__main__':
    X_r, Y_r, Psi_r, xr_dot, psi_r_dot = path_generator()

    # for i in range(len(xr_dot)):
    for i in range(1):
        print  ("%d th loop" %i)
        car = state()
        # This is Outer Loop (Kinematic) per 0.1 s --> Evaluate position
        if i == 0 :
            X_k = 0
            Y_k = 0
            Psi_k = 0
            x_dot = 0.1
            y_dot = 0
            psi_dot = 0
            delta_steer = 0
            a = 0
        else : 
            X_k = X_r[i] - next_k_state[0,0]
            Y_k = Y_r[i] - next_k_state[1,0]
            Psi_k = Psi_r[i] - next_k_state[2,0]
            # x_dot = next_U_k[0,0]
            # psi_dot = next_U_k[1,0]
            # x_dot = U_cd [0]
            psi_dot = x_dot*np.tan(U_cd[1])/(lf+lr)

            print("=======================")
            print("omega : ", U_k[1,0])
            delta_steer= np.arctan(U_k[1,0]*(lf+lr)/U_k[0,0])
            print("delta : ", delta_steer)
            print("=======================")

        iteration.append(i)

        ref_pos_x.append(X_r[i])
        ref_pos_y.append(Y_r[i])
        ref_psi.append(Psi_r[i])
        ref_x_dot.append(xr_dot[i][0])
        ref_psi_dot.append(psi_r_dot[i][0])

        car_pos_x.append(X_k)
        car_pos_y.append(Y_k)
        car_psi.append(Psi_k)
        car_delta.append(delta_steer)

        car_x_dot.append(x_dot)
        car_psi_dot.append(psi_dot)

        # === kinematic control =====
        car.x = X_r[i] - X_k
        car.y = Y_r[i] - Y_k
        car.psi = Psi_r[i] - Psi_k
        # === kinematic reference =====
        car.x_dot_ref = xr_dot[i]
        car.psi_dot_ref = psi_r_dot[i] 
        # === dinamic control =====
        car.x_dot = x_dot
        car.y_dot = y_dot
        car.psi_dot = psi_dot
        car.delta = delta_steer
        # print("===========================")
        # print("X_eror : ",X_r[i], " - ", X_k, " = ", car.x)
        # print("Y_error : ", Y_r[i], " - ", Y_k, " = ", car.y)
        # print("Psi_error : ",Psi_r[i], " - ", Psi_k, " = ", car.psi)
        # print("Vd  : ", car.x_dot_ref)
        # print("W omega : ", car.psi_dot_ref)
        # print("===========================")
        next_k_state, U_k = kinematic_control(car)
        # next_d_state, U_d = dynamic_control (car)

        setpoint_dynamic = np.array([[U_k[0,0]], [0], [U_k[1,0]]])
        print("======= Ini Loop Dynamic ===========")
        print("Setpoint Dynamic : ", setpoint_dynamic)

        # delta_car = np.arctan(U_k[1,0]*(lf+lr)/U_k[0,0]) + U_d[0,0]
        # a = U_d[1,0]
        # x_dot_car = U_k[0,0] + a*Td
        # U_cd = [delta_car, x_dot_car]


        for i in range(5):
            # psi_dot = np.tan(U_cd[0])*U_cd[1]/(lf+lr)
            print("===========================")
            print("loop ke : ", i)
            print("Linear velocity : ", x_dot ) #U_cd[1]
            print("Angular Velocity : ", psi_dot)
            # car.x_dot = U_cd[1]
            car.x_dot = x_dot
            car.y_dot = y_dot
            car.psi_dot = psi_dot
            next_d_state, U_d =  dynamic_control (car)

            delta_steer = U_d[0,0]
            a = U_d[1,0]
            print("Steering Angle : ", delta_steer) #U_cd[0]
            print("Acceleration : ", a)
            print("===========================")
            x_dot = next_d_state[0,0]
            y_dot = next_d_state[1,0]
            psi_dot = next_d_state[2,0]

        print("======= Hasil Kinematic ==========")
        print("Car linear velocity", U_k[0,0])
        print("Angular Velocity", U_k[1,0])
        # print("Car steering", delta_car)
        print("======= Hasil Dinamic ==========")
        print("Car linear velocity", x_dot)
        print("Angular Velocity", psi_dot)
        print("Car steering", delta_steer)
        print("===========================")

        # next_car_state = calculate_new_states()

        # time.sleep(1)
        i = i+1

    # print("Done")
    # # print("Car Position (m)")
    # # print("X: ", car_pos_x)
    # # print("Y: ", car_pos_y)
    # # print("Angle : ", car_psi)
    # # print("Delta Steering : ", car_delta)

    # # print("Dimensi X", len(car_pos_x))
    # # print("Dimensi X_ref", len(ref_pos_x))
    # # print("Dimensi Y", len(car_pos_y))
    # # print("Dimensi Y_ref", len(ref_pos_y))
    # # print("Dimensi Psi", len(car_psi))
    # # print("Dimensi Psi_ref", len(ref_psi))

    # print("Dimensi X_dot", len(car_x_dot))
    # print("Dimensi X_dot_ref", len(xr_dot))
    # print("Dimensi Psi_dot", len(car_psi_dot))
    # print("Dimensi Psi_dot_ref", len(psi_r_dot))

    # # ====== PLOT TRAJECTORY =====
    # plt.plot(ref_pos_x,ref_pos_y,'b',linewidth=2,label='The trajectory')
    # plt.plot(car_pos_x,car_pos_y,'--r',linewidth=2,label='Car position')
    # plt.xlabel('x-position [m]',fontsize=15)
    # plt.ylabel('y-position [m]',fontsize=15)
    # plt.grid(True)
    # plt.legend(loc='upper right',fontsize='small')
    # plt.ylim(-ref_pos_x[-1]/2,ref_pos_x[-1]/2) # Scale roads (x & y sizes should be the same to get a realistic picture of the situation)
    # plt.show()

    # # Membuat subplot pertama
    # plt.subplot(3, 1, 1)  # 3 baris, 1 kolom, subplot pertama
    # plt.title('Longitudinal Velocity')
    # plt.plot(iteration,ref_x_dot,'--r',linewidth=2,label='Setpoint')
    # plt.plot(iteration,car_x_dot,'b',linewidth=2,label='Car')
    # plt.xlabel('Iterasi')
    # plt.ylabel('Vx')
    # plt.legend(loc='upper right',fontsize='small')

    # # Membuat subplot kedua
    # plt.subplot(3, 1, 2)  # 3 baris, 1 kolom, subplot kedua
    # plt.title('Angular Velocity (Rad/s)')
    # plt.plot(iteration,ref_psi_dot,'--r',linewidth=2,label='Setpoint')
    # plt.plot(iteration,car_psi_dot,'b',linewidth=2,label='Car')
    # plt.xlabel('Iterasi')
    # plt.ylabel('Omega')
    # plt.legend(loc='upper right',fontsize='small')

    # # Membuat subplot ketiga
    # plt.subplot(3, 1, 3)  # 3 baris, 1 kolom, subplot ketiga
    # plt.title('Steering Angle')
    # plt.plot(iteration,car_delta,'b',linewidth=2,label='Car Steering')
    # plt.xlabel('Iterasi')
    # plt.ylabel('delta')
    # plt.legend(loc='upper right',fontsize='small')

    # # Menyesuaikan layout
    # plt.tight_layout()

    # # Menampilkan grafik
    # plt.show()




