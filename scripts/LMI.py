#!/usr/bin/env python3
import cvxpy as cp
import numpy as np
from constants import *
from function import Kinematic

Ac_pk, Bc = Kinematic.getModel()

# print("Ac_pk : ", Ac_pk)
print("Bc : ", Bc)

invQts= np.linalg.inv(Q_ts)
invRts= np.linalg.inv(R_ts)
Y = cp.Variable((n, n),PSD=True)
W = [cp.Variable((m, n)) for _ in range(2**n)]

LQR_param = [cp.Variable((n, n)) for _ in range(2**n)]


constraints = []
# constraints = [cp.lambda_min(Y)>=0.01]
# constraints = [Y>=0.001*np.ones([n,n])]
# constraints += [lamda == cp.trace(Y)]

# constraints = [cp.lambda_min(Y) >=0.1]
# print(0.3*np.eye(11))
# for i in range (2**n):
for i in range(1):
    lmi_prob = cp.vstack([
        cp.hstack([Y, (Ac_pk[i] @ Y + Bc @ W[i]).T ,Y, W[i].T]), #baris 1
        cp.hstack([(Ac_pk[i] @ Y + Bc @ W[i]), Y, np.zeros([3,3]), np.zeros([3,2])]), #baris 2
        cp.hstack([Y, np.zeros([3,3]), invQts, np.zeros([3,2])]), #baris 3
        cp.hstack([W[i], np.zeros([2,3]), np.zeros([2,3]), invRts]) #baris 4
        ])
    

    # constraints += [lmi_prob>>0]
    constraints += [lmi_prob>=(0.1*np.eye(11))] # lmi definit positif dgn batasan lebih spesifik agar nilai Y dan Wi tidak nol
    # constraints += [cp.lambda_min(lmi_prob)>=0.0001]

    # lmi_prob = Ac_pk[i]@Y + Y@Ac_pk[i] - (Bc@W[i]).T - Bc@W[i]
    # LQR_param[i] = Y@Q_ts@Y + W[i].T@R_ts@W[i]
    # lmi_prob2 = np.matmul(np.matmul(Y,Q_ts),Y) + np.matmul(np.matmul(W[i].T,R_ts),W[i])
    # constraints += [lmi_prob<<0]
    # constraints += [LQR_param[i]<<0]

obj = cp.Minimize(0)
problem = cp.Problem(obj, constraints)
problem.solve(solver=cp.SCS, qcp=False, verbose = False,max_iters=1000)
if problem.status == cp.OPTIMAL:
    print("Optimal value", problem.value)
    # print("lmi_prob : ", lmi_prob)
    # print("Y", Y.value)
    # print("W", [w.value for w in W])
    outputKi= np.zeros([8,2,3])
    outputW= np.zeros([8,2,3])
    y_opt = Y.value
    # print("Eigenvalue Y = ", np.linalg.eigvals(y_opt))
    P = np.linalg.inv(y_opt)
    # print("Y_inverse = ", P)
    # print("Eigenvalue Y_inverse = ", np.linalg.eigvals(P))
    for i in range(1):
        outputKi[i]=W[i].value @ P
        outputW[i]=W[i].value
        print("Y", y_opt)
        print("Output W", outputW[i])
else:
    print("Problem not solved")
    print("Status:", problem.status)

print("outputKi[0]", outputKi[0])

# for i in range(2**n):


# for i in range (2**n):
#     # Konversi ke array numpy
#     Y = y_opt
#     W[i] = outputW[i]

#     # Definisikan matriks-matriks yang akan digabungkan
#     row1 = np.concatenate((Y, (Ac_pk[i] @ Y + Bc @ W[i]).T, Y, W[i].T), axis=1)
#     row2 = np.concatenate((Ac_pk[i] @ Y + Bc @ W[i], Y, np.zeros((3, 3)), np.zeros((3, 2))), axis=1)
#     row3 = np.concatenate((Y, np.zeros((3, 3)), invQts, np.zeros((3, 2))), axis=1)
#     row4 = np.concatenate((W[i], np.zeros((2, 3)), np.zeros((2, 3)), invRts), axis=1)

#     # Gabungkan baris-baris menjadi matriks tunggal
#     lmi_prob = np.concatenate((row1, row2, row3, row4), axis=0)
#     # print("lmi_prob", lmi_prob.shape)
#     # print("Eigen values", np.linalg.eigvals(lmi_prob))
#     eigenvalues = np.linalg.eigvals(lmi_prob)
#     if np.all(eigenvalues >= 0):
#         print("positif semidefinit.")
#     else:
#         print("TIDAK matriks positif semidefinit.")



    
"""
#================================================================================================

Z = cp.Variable((n, n))
u_bar = u_max
u_bar_squared= u_bar@u_bar.T
S= np.zeros([3,3])

# inv_u_bar_squared = np.linalg.inv(u_bar_squared)
# print("inv_u_bar_squared : ", inv_u_bar_squared)
# eigenvalues = np.linalg.eigvals(inv_u_bar_squared)
# if np.all(eigenvalues >= 0):
#     print("inv_u_bar_squared adalah matriks positif semidefinit.")
# else:
#     print("inv_u_bar_squared bukan matriks positif semidefinit.")

constraints2=[]
# constraints2 += [cp.lambda_min(Z) >=0.01, cp.lambda_max(Z)<=2]
# constraints2 += [cp.lambda_sum_smallest(Z, 3) >= 0.1]
for i in range (2**n):
    Ai = Ac_pk[i]
    Ki = outputKi[i]
    lmi_prob = cp.vstack([
        cp.hstack([-Z, Z@(Ai+Bc@Ki).T]), #baris 1
        cp.hstack([(Ai+Bc@Ki)@Z, -Z]), #baris 2
        ])
    constraints2 += [lmi_prob<<0]
    constraints2 += [Ki@Z@Ki.T-u_bar_squared<<0]

    # lmi_prob2 = cp.vstack([
    #     cp.hstack([Ki@Z@Ki.T, np.diag([1,1])]), #baris 1
    #     cp.hstack([np.diag([1,1]), inv_u_bar_squared]), #baris 2
    #     ])
    # constraints2 += [lmi_prob2<<0]
# constraints2 += [cp.lambda_sum_largest(Z,4)]
obj2 = cp.Maximize(1)
problem2 = cp.Problem(obj2, constraints2)
problem2.solve(solver=cp.SCS, verbose=False,max_iters=1000)
if problem2.status == cp.OPTIMAL:
    print ("Z : ", Z.value)
    S=np.linalg.inv(Z.value)
else:
    print("Problem not solved")
    print("Status:", problem2.status)
    
# print("Output P", P)
# print("Output Ki", outputKi)

# INI MATRIX S DARI JURNAL REFERENSI
S = np.array([
    [0.465, 0, 0],
    [0, 23.813, 76.596],
    [0, 76.596, 257.251]
])  

print("S", S)
eigenvalues = np.linalg.eigvals(P)
if np.all(eigenvalues >= 0):
    print("P adalah matriks positif semidefinit.")
else:
    print("P bukan matriks positif semidefinit.")

eigenvalues = np.linalg.eigvals(S)
if np.all(eigenvalues >= 0):
    print("S adalah matriks positif semidefinit.")
else:
    print("S bukan matriks positif semidefinit.")


"""