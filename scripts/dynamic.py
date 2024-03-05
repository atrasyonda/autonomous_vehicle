#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib as plt
import random
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
from constants import *


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
    Bd= np.array([
        [0, Td],
        [Td*Caf/m, 0],
        [Td*Caf*lf/I, 0]
    ])
    log_message = "Nilai x_dot: %s, Nilai y_dot: %s, Nilai delta: %s" % (x_dot, y_dot,delta)
    rospy.loginfo(log_message)
    return Ad,Bd

if __name__=='__main__':
    rospy.init_node("dynamic_node")
    rospy.loginfo("Node has been started")
    d_state = rospy.Subscriber("/car/state", state, callback=getDynamic)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()