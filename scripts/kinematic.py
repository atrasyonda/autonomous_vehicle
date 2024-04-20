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

    # Ac = Kinematic.getLPV(data.psi_dot, data.x_dot_ref[0], data.psi, Ac_pk)  # get LPV model
    # next_state = Ac@X_k + Bc@control_signal - Bc@Rc_k[:,:,0]
    
    print("=====================================")
    print("next_state : ", next_state)
    print("control_signal : ", control_signal)
    # pub.publish(state(x = next_state[0,0], y = next_state[1,0], psi = next_state[2,0], 
    #                   x_dot = control_signal[0,0], psi_dot = control_signal[0,0]))



if __name__=='__main__':
    rospy.init_node("kinematic_node")
    rospy.loginfo("Node has been started")
    k_state = rospy.Subscriber("/car/state", state, callback)
    # pub = rospy.Publisher("/car/next_state", state, queue_size=10 )
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()




  