#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
from constants import *
from function import *

c = xr_dot_min*np.sin(psi_max)*Tc/psi_max


# Y = np.array([
#     [0.0001, 0.0000, 0.0000],
#     [0.0000, 0.0012,-0.0003],
#     [0.0000,-0.0003, 0.0001]
# ])

Y = np.array([
    [0.0002,   -0.0000,   -0.0000],
    [-0.0000,    0.7772,    0.7918],
    [-0.0000,    0.7918,    0.8097]
])
    
print(Y)
eigen = np.linalg.eigvals(Y)
print("Eigenvalue", eigen)

P = np.linalg.inv(Y)
print("P", P)