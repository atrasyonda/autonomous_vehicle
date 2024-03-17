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
Kinematic.MPC(Ac_pk[0], Bc, P, Ki, S)  # get MPC model


def callback(data):
    x_k = np.array([[data.x], [data.y], [data.psi]])
    print("state : ", x_k)
    Ac = Kinematic.getLPV(data, Ac_pk)  # get LPV model


if __name__=='__main__':
    rospy.init_node("kinematic_node")
    rospy.loginfo("Node has been started")
    k_state = rospy.Subscriber("/car/state", state, callback)
    # pub = rospy.Publisher("/car/model", Twist, queue_size=10 )
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()




  