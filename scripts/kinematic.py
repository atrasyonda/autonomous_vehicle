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
a = 0

def callback(data):
    print (data)
    x_k = np.array([[data.x], [data.y], [data.psi]])
    u_k = np.array([[data.delta], [a]])
    x_dot_psi_e = [x * data.psi for x in data.x_dot_ref]
    
    rc_k = np.array([[x_dot_psi_e], [data.psi_dot_ref]])
    # print (rc_k.shape)

    # print("state : ", x_k.shape)
    Ac = Kinematic.getLPV(data, Ac_pk)  # get LPV model
    x_opt, deltau_opt, u_opt = Kinematic.MPC(x_k, u_k, rc_k, Ac, Bc, P, S)

    # ======= Calculate next state ========
    control_signal = u_opt[0,:]
    next_state = Ac@x_k + Bc@control_signal - Bc@rc_k[:,:,0]
    # print("next_x_opt : ", x_opt[1])
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




  