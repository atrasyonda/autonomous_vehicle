#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib as plt
import random
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
from function import Kinematic

Ac_pk, Bc = Kinematic.getModel()  # get model parameters
P, Ki, S= Kinematic.getMPCSet(Ac_pk,Bc) 
# set initial variabel

def callback(data):
    # ========= Extract the data =================
    x_e = data.x
    y_e = data.y
    psi_e = data.psi

    x_dot = data.x_dot
    psi_dot = data.psi_dot

    xr_dot = data.x_dot_ref
    psi_r_dot = data.psi_dot_ref
    # Construct Vector of Schedulling Variables
    pk = [psi_dot, xr_dot, psi_e]
    # print("pk : ", pk)
    
    # Construct the State-Space model
    X_k = np.array([[x_e], [y_e], [psi_e]])  # get current error state 
    U_k = np.array([[x_dot], [psi_dot]]) # get previous control signal

    # Construct reference signal for N horizon prediction
    xr_dot_psi_e = [x * data.psi for x in psi_r_dot]

    
    next_x_opt, u_opt = Kinematic.LPV_MPC(X_k, U_k, pk, Ac_pk, Ac, Bc, P, S)

    # ======= Calculate next state ========
    control_signal = u_opt
    Ac = Kinematic.getLPV(psi_dot, xr_dot[0], psi_e, Ac_pk)  # get LPV model
    Rc_k = np.array([[xr_dot_psi_e], [psi_r_dot]])

    next_state = Ac@X_k + Bc@control_signal - Bc@Rc_k[:,:,0]
    
    print("=====================================")
    print("next_state : ", next_state)
    print("control_signal : ", control_signal)
    pub.publish(state(x = next_state[0,0], y = next_state[1,0], psi = next_state[2,0], 
                      x_dot = control_signal[0,0], psi_dot = control_signal[0,0]))



if __name__=='__main__':
    rospy.init_node("kinematic_node")
    rospy.loginfo("Node has been started")
    k_state = rospy.Subscriber("/car/state", state, callback)
    pub = rospy.Publisher("/car/next_state", state, queue_size=10 )
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()




  