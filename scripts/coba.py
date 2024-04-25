#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
# from geometry_msgs.msg import Twist
from autonomous_vehicle.msg import state
from constants import *
from function import *

c = xr_dot_min*np.sin(psi_max)*Tc/psi_max

print(c)