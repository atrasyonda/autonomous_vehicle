#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib as plt
import random
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
# from constants import *
from function import Dynamic

Ad_vk, Bd = Dynamic.getModel()
Ki = Dynamic.getLQR(Ad_vk, Bd)

def callback(data):
    x_d = np.array([[data.x_dot], [data.y_dot],[data.psi_dot]]) 
    miu_vk, Ad = Dynamic.getLPV(data, Ad_vk)
    K_vk = Dynamic.evaluateLQR (miu_vk, Ki)
    Ad_cl = Ad - Bd@K_vk
    
    print("Ad_vk", Ad_vk)
    print ("Ad", Ad)
    print("Ad closed loop", Ad_cl)

if __name__=='__main__':
    rospy.init_node("dynamic_node")
    rospy.loginfo("Node has been started")
    d_state = rospy.Subscriber("/car/state", state, callback)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()