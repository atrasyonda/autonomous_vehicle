#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib as plt
import random
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
from constants import *
from function import LPV, LMI
# variabel state dari hasil pembacaan sensor / state sebelumnya



def callback(data):
    Ac,Ac_pk,Bc = LPV.getKinematic(data)
    P,Ki = LMI.getMPCSet(Ac_pk,Bc)
    # print(np.linalg.eig(Ac)[0])
    # print(Ac)
    print(Bc)
    # print(Ac_pk)
    # print("P =  " , P)
    # print("Ki=  " , Ki)
    print("=================================")


if __name__=='__main__':
    rospy.init_node("kinematic_node")
    rospy.loginfo("Node has been started")
    k_state = rospy.Subscriber("/car/state", state, callback)
    # pub = rospy.Publisher("/car/model", Twist, queue_size=10 )
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()




  