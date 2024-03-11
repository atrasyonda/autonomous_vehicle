#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib as plt
import random
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
from constants import *
from function import LPV, LMI

def callback(data):
    Ad,Ad_vk,Bd,miu_vk = LPV.getDynamic(data)
    # print("Ad_vk = ", Ad_vk)
    for i, element in enumerate(Ad_vk):
        print ("A", i, " = ", element)
    print("=================================")

if __name__=='__main__':
    rospy.init_node("dynamic_node")
    rospy.loginfo("Node has been started")
    d_state = rospy.Subscriber("/car/state", state, callback)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()