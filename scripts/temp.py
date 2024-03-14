import cvxpy as cp
import numpy as np

n = 3  # number of states
m = 2  # number of inputs
# Ai=np.random.randint(1,5,(3,3)) #state matrix --> Jumlah Ai nanti ada 8 (sesuai Ac_pk A0 - A7)
Bc = np.array([[-0.1,0],[0,0],[0,-0.1]]) # input matrix
Qts = np.diag([1,1,3])  # state weight matrix --> JURNAL
Rts = np.diag([1,3])  # input weight matrix --> JURNAL
invRts= np.linalg.inv(Rts)
invQts= np.linalg.inv(Qts)
Y = cp.Variable((n, n),symmetric=True)
Wi = cp.Variable((m, n)) # Solusi bobot untuk kontroler bds Ai --> nanti dibuat loop untuk dapat W0 - W7

A0= np.array([[1, -0.142, 0], [0.142, 1, 0.00999583], [0, 0, 1]])
A1= np.array([[ 1, -0.142, 0], [0.142, 1, 0.00999583], [0, 0, 1]])
A2= np.array([[1, -0.142, 0], [0.142, 1, 1.99916677], [0, 0, 1]])
A3= np.array([[1, -0.142, 0,], [0.142, 1, 1.99916677], [0, 0, 1,]])
A4= np.array([[1, 0.142, 0], [-0.142, 1, 0.00999583], [0, 0, 1]])
A5= np.array([[1, 0.142, 0], [-0.142, 1, 0.00999583], [0, 0, 1]])
A6= np.array([[1, 0.142, 0], [-0.142, 1, 1.99916677], [0, 0, 1]])
A7= np.array([[1, 0.142, 0,], [-0.142, 1, 1.99916677], [0, 0, 1,]])

Ac_pk = [A0, A1, A2, A3, A4, A5, A6, A7]

outputKi= np.zeros([8,2,3])

for i in range (2**n):
    # print(f"Indeks {i}: {element}")
    Ai = Ac_pk[i]
    if i == 0 :
        lmi = cp.vstack([
            cp.hstack([Y, (Ai@Y+Bc@Wi).T ,Y, Wi.T]), #baris 1
            cp.hstack([(Ai@Y+Bc@Wi), Y, np.zeros([3,3]), np.zeros([3,2])]), #baris 2
            cp.hstack([Y, np.zeros([3,3]), invQts, np.zeros([3,2])]), #baris 3
            cp.hstack([Wi, np.zeros([2,3]), np.zeros([2,3]), invRts]) #baris 4
            ])
        constraints = [lmi>=(0.3*np.eye(11)), Y>>0] # lmi definit positif dgn batasan lebih spesifik agar nilai Y dan Wi tidak nol
        obj = cp.Minimize(0)
        problem = cp.Problem(obj, constraints)
        problem.solve(solver=cp.SCS)
        if problem.status == cp.OPTIMAL:
            y_opt = Y.value
            P = np.linalg.inv(y_opt)
            Ki = Wi.value@P
            outputKi[i]=Ki
        else:
            print("Problem not solved")
            print("Status:", problem.status)
    else :
        lmi = cp.vstack([
            cp.hstack([y_opt, (Ai@y_opt+Bc@Wi).T ,y_opt, Wi.T]), #baris 1
            cp.hstack([(Ai@y_opt+Bc@Wi), y_opt, np.zeros([3,3]), np.zeros([3,2])]), #baris 2
            cp.hstack([y_opt, np.zeros([3,3]), invQts, np.zeros([3,2])]), #baris 3
            cp.hstack([Wi, np.zeros([2,3]), np.zeros([2,3]), invRts]) #baris 4
            ])
        constraints = [lmi>=(0.3*np.eye(11))] # lmi definit positif dgn batasan lebih spesifik agar nilai Y dan Wi tidak nol
        obj = cp.Minimize(0)
        problem = cp.Problem(obj, constraints)
        problem.solve(solver=cp.SCS)
        if problem.status == cp.OPTIMAL:
            Ki = Wi.value@P
            outputKi[i]=Ki
        else:
            print("Problem not solved")
            print("Status:", problem.status)
            outputKi[i]=0
print("Output P", P)
print("Output Ki", outputKi)

#================================================================================================
Z = cp.Variable((n, n), symmetric=True)
u_bar= np.array([[1.4], [20]]) # matrix 2x1
u_bar_squared= u_bar@u_bar.T

S= np.zeros([3,3])
lmi2=[]

for i in range (2**n):
    Ai = Ac_pk[i]
    Ki = outputKi[i]
    lmi_prob = cp.vstack([
        cp.hstack([-Z, Z@(Ai+Bc@Ki).T]), #baris 1
        cp.hstack([(Ai+Bc@Ki)@Z, -Z]), #baris 2
        ])
    lmi2.append(lmi_prob)

# print ("lmi2 ke 1", lmi2[0])
constraints2 = [lmi2[0]<<0, 
                lmi2[1]<<0,
                lmi2[2]<<0,
                lmi2[3]<<0,
                lmi2[4]<<0,
                lmi2[5]<<0,
                lmi2[6]<<0,
                lmi2[7]<<0,
                Ki@Z@Ki.T-u_bar_squared<<0] 
obj2 = cp.Maximize(0)
problem2 = cp.Problem(obj2, constraints2)
problem2.solve(solver=cp.SCS)
if problem2.status == cp.OPTIMAL:
    S=np.linalg.inv(Z.value)
else:
    print("Problem not solved")
    print("Status:", problem2.status)
print ("Output S:" , S)



# VERSI ASLI --> Semua solusi ada 8 buah matrix , harusnya Y dan Z cuma 1 matrix

#========================== Terminal Weighting Matrix
# outputKi= np.zeros([8,2,3])
# outputP= np.zeros([8,3,3])

# for i in range (2**n):
#     # print(f"Indeks {i}: {element}")
#     Ai = Ac_pk[i]
#     lmi = cp.vstack([
#         cp.hstack([Y, (Ai@Y+Bc@Wi).T ,Y, Wi.T]), #baris 1
#         cp.hstack([(Ai@Y+Bc@Wi), Y, np.zeros([3,3]), np.zeros([3,2])]), #baris 2
#         cp.hstack([Y, np.zeros([3,3]), invQts, np.zeros([3,2])]), #baris 3
#         cp.hstack([Wi, np.zeros([2,3]), np.zeros([2,3]), invRts]) #baris 4
#         ])
#     constraints = [lmi>=(0.3*np.eye(11)), Y>>0] # lmi definit positif dgn batasan lebih spesifik agar nilai Y dan Wi tidak nol
#     obj = cp.Minimize(0)
#     problem = cp.Problem(obj, constraints)
#     problem.solve(solver=cp.SCS)

#     if problem.status == cp.OPTIMAL:
#         P = np.linalg.inv(Y.value)
#         Ki = Wi.value@P
#         outputP[i]=P
#         outputKi[i]=Ki
#     else:
#         print("Problem not solved")
#         print("Status:", problem.status)

#=====================  Terminal Region ==============
# for i in range (2**n):
#     Ai = Ac_pk[i]
#     Ki = outputKi[i]
#     lmi2 = cp.vstack([
#         cp.hstack([-Z, Z@(Ai+Bc@Ki).T]), #baris 1
#         cp.hstack([(Ai+Bc@Ki)@Z, -Z]), #baris 2
#         ])
#     constraints2 = [lmi2<<0, Ki@Z@Ki.T-u_bar_squared<<0] 
#     obj2 = cp.Maximize(0)
#     problem2 = cp.Problem(obj2, constraints2)
#     problem2.solve(solver=cp.SCS)
#     if problem2.status == cp.OPTIMAL:
#         print("Optimal value:", problem2.value)
#         S[i]=np.linalg.inv(Z.value)
#     else:
#         print("Problem not solved")
#         print("Status:", problem2.status)
# print ("Output S:" , S)

"""
Q = 0.9*np.diag([0.33, 0.33, 0.33]) # state weight matrix Q 3x3
R = 0.1*np.diag([0.8, 0.2]) # input weight matrix R 2x1
N = 20 # horizon
delta_u = np.ones((m, 1, N)) # 2x1x20
x = np.ones((n,1, N+1)) # 3x1x21

# print (np.shape(x[:, 0]))

# matrix_3d = np.random.rand(m, 1, N+1)
# print("Matrix 3D:")
# print(matrix_3d[:,:,1])
# print("Shape:", matrix_3d[:,:,1].shape)

def cost_function(delta_u, x, Q, R, P, N): # Jk for MPC cost function
    cost = 0
    for i in range(N):
        cost += x[:,:, i].T@(Q@x[:,:, i]) + delta_u[:,:,i].T@(R @ delta_u[:,:,i])
    cost += x[:,:,N].T @(P@x[:,:, N])
    return cost
Jk = cost_function(delta_u, x, Q, R, P, N)
print ("Jk = ", Jk)
"""

