import rospy
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt

from function import Kinematic
from constants import *

Ac_pk, Bc = Kinematic.getModel()  # get model parameters
P, Ki, S= Kinematic.getMPCSet(Ac_pk,Bc) 
# set initial variabel

# S = np.array([
#     [0.465, 0, 0],
#     [0, 23.813, 76.596],
#     [0, 76.596, 257.251]
# ])  
# print("S : ", S)

Z = np.linalg.inv(S)
print("Z : ", Z)

eigenvalue = np.linalg.eigvals(Z)

print(eigenvalue)
"""

S = np.linalg.inv(Z)
print("S : ", S)

outputKi = [cp.Variable((m, n)) for _ in range(2**n)]

u_bar = u_max
u_bar_squared= u_bar@u_bar.T
print("u_bar_squared : ", u_bar_squared)
constraints2=[]
for i in range (2**n):
    Ai = Ac_pk[i]
    Ki = outputKi[i]
    lmi_prob = cp.vstack([
        cp.hstack([-Z, Z@(Ai+Bc@Ki).T]), #baris 1
        cp.hstack([(Ai+Bc@Ki)@Z, -Z]), #baris 2
        ])
    constraints2 += [lmi_prob<<0]
    lmi_prob2 = cp.vstack([
        cp.hstack([u_bar_squared, Ki]), #baris 1
        cp.hstack([Ki.T, np.linalg.inv(Z)]), #baris 2
        ])
    constraints2 += [lmi_prob2>>0]

obj2 = cp.Maximize(0)
problem2 = cp.Problem(obj2, constraints2)
problem2.solve(solver=cp.SCS)
if problem2.status == cp.OPTIMAL:
    print("Optimal value", problem2.value)
    Ki = np.zeros([8,2,3])
    for i in range(2**n):
        Ki[i] = outputKi[i]
else:
    print("Problem not solved")
    print("Status:", problem2.status)
"""