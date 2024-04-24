#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import random
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
from constants import *
from function import Dynamic
from lmi_param import K_d

Ad_vk, Bd = Dynamic.getModel()


delta = np.linspace(delta_min, delta_max, 10)
x_dot = np.linspace(x_dot_min, x_dot_max, 10)
y_dot = np.linspace(y_dot_min, y_dot_max, 10)

psi_dot = np.linspace(psi_dot_min, psi_dot_max, 10)

openloop_eigen = np.zeros([10,3])
cleseloop_eigen = np.zeros([10,3])

reference = np.array([[-1], [19]]) # [Psidot , Vx]
Kr = 1
# print(reference)
for i in range(10):
    vk = [delta[i], x_dot[i], y_dot[i]]
    X_d = np.array([[x_dot[i]], [y_dot[i]],[psi_dot[i]]])
    Ad, K_vk, Miu_vk = Dynamic.LPV_LQR(vk, Ad_vk, K_d)
    # print("Loop Ke -", i)
    # print("=========================================")
    # print(Ad)
    # print("=========================================")
    # print(K_vk)
    print("=========================================")
    eigenvalue = np.linalg.eigvals(Ad)
    print("Open-loop Eigenvalue", eigenvalue)
    openloop_eigen[i] = eigenvalue
    eigenvalue = np.linalg.eigvals(Ad-Bd@K_vk)
    cleseloop_eigen[i] = eigenvalue
    print("Closed-loop Eigenvalue", eigenvalue)
    print("=========================================")

print("Open-loop Eigenvalue", openloop_eigen)
print("Closed-loop Eigenvalue", cleseloop_eigen)
