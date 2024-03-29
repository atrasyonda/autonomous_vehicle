#!/usr/bin/env python3
import rospy
import random
from autonomous_vehicle.msg import state
import matplotlib.pyplot as plt
from constants import *
from function import Kinematic

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
        
    print ("X = ", x)
    print ("Y = ", y)
    print ("PSI = ", psiInt)
    print("X_dot = ", dx)
    print("PSI_dot = ", dpsi)

    print ("Dimensi X = ", len(x))
    print ("Dimensi Y = ", len(y))
    print ("Dimensi PSI = ", len(psiInt))
    print ("Dimensi X_dot", len(dx))
    print ("Dimensi PSI_dot", len(dpsi))


    # # Plot the world
    # plt.plot(x,y,'b',linewidth=2,label='The trajectory')
    # # plt.plot(x,statesTotal[:,3],'--r',linewidth=2,label='Car position')
    # plt.xlabel('x-position [m]',fontsize=15)
    # plt.ylabel('y-position [m]',fontsize=15)
    # plt.grid(True)
    # plt.legend(loc='upper right',fontsize='small')
    # plt.ylim(-x[-1]/2,x[-1]/2) # Scale roads (x & y sizes should be the same to get a realistic picture of the situation)
    # plt.show()
    return x,y,psiInt,dx,dpsi

def odom_publisher (_k, i) :
    if i == 0 :
        x = 0
        y = 0
        psi = 0
        x_dot = 0
        y_dot = 0
        psi_dot = 0
        delta = 0
    else :
        x = x + x_dot * Tc
        y = y + y_dot * Tc
        psi = psi + psi_dot * Tc

if __name__=='__main__':
    X_r, Y_r, Psi_r, xr_dot, psi_dot = path_generator()
    rospy.init_node("state_publisher")
    rospy.loginfo("Node has been started")
    pub = rospy.Publisher("/car/state", state, queue_size=10 )
    k_state = rospy.Subscriber("/car/next_state", state, callback=odom_publisher)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        car = state()
        # === kinematic control =====
        car.x_e = round(random.uniform(0, 5),2)
        car.y_e = round(random.uniform(0, 5),2)
        car.psi_e = round(random.uniform(psi_min, psi_max),2) # also for kinematic scheduling variable
        # for reference r_c
        car.x_dot_ref = round(random.uniform(xr_dot_min, xr_dot_max),2) # also for kinematic scheduling variable
        car.psi_dot_ref = round(random.uniform(0, 5),2)
        # === dynamic control ======
        car.x_dot=round(random.uniform(x_dot_min, x_dot_max),2) # also for dynamic scheduling variable
        car.y_dot=round(random.uniform(y_dot_min, y_dot_max),2) # also for dynamic scheduling variable
        car.psi_dot = round(random.uniform(psi_dot_min, psi_dot_max),2) # also for kinematic scheduling variable
        
        car.delta=round(random.uniform(delta_min, delta_max),2) # also for dynamic scheduling variable

        pub.publish(car)

        log_message = "x_e: %s, y_e: %s, psi_e: %s,x_dot_ref: %s, psi_dot_ref: %s, x_dot: %s, y_dot: %s, psi_dot: %s, delta: %s" % (car.x_e, car.y_e,car.psi_e, car.x_dot_ref,car.psi_dot_ref,car.x_dot,car.y_dot,car.psi_dot,car.delta)
        rospy.loginfo(log_message)
        rate.sleep()