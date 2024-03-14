import cvxpy as cp
import numpy as np

# menambahkan baris
n = 3  # number of states
m = 2  # number of inputs
# Ai=np.random.randint(1,5,(3,3)) #state matrix --> Jumlah Ai nanti ada 8 (sesuai Ac_pk A0 - A7)
Bd = np.array([[-0.1,0],[0,0],[0,-0.1]]) # input matrix
Qts = 0.9*np.diag([0.66, 0.01, 0.33])  # state weight matrix --> JURNAL
Rts = 0.1*np.diag([0.5, 0.5])  # input weight matrix --> JURNAL
invRts= np.linalg.inv(Rts)
invQts= np.linalg.inv(Qts)
Y = cp.Variable((n, n),symmetric=True)
Wi = cp.Variable((m, n)) # Solusi bobot untuk kontroler bds Ai --> nanti dibuat loop untuk dapat W0 - W7

A0 = np.array([[0.43896, -247.10937926, -24.72093793],
               [0.0, -370.31859311, 94.86714069],
               [0.0, 23.71703517, 3.37170352]])
A1 = np.array([[0.43896, -247.10937926, -24.70093793],
               [0.0, -370.31859311, 94.86714069],
               [0.0, 23.71703517, 3.37170352]])
A2 = np.array([[-13.21045, -1.2355469, -0.13355469],
               [0.0, -0.85659297, 0.2743407],
               [0.0, 0.11858518, 1.01185852]])
A3 = np.array([[-13.21045, -1.2355469, -0.11355469],
               [0.0, -0.85659297, 0.2743407],
               [0.0, 0.11858518, 1.01185852]])
A4 = np.array([[0.43896, 247.10937926, 24.70093793],
               [0.0, -370.31859311, 94.86714069],
               [0.0, 23.71703517, 3.37170352]])
A5 = np.array([[0.43896, 247.10937926, 24.72093793],
               [0.0, -370.31859311, 94.86714069],
               [0.0, 23.71703517, 3.37170352]])
A6 = np.array([[-13.21045, 1.2355469, 0.11355469],
               [0.0, -0.85659297, 0.2743407],
               [0.0, 0.11858518, 1.01185852]])
A7 = np.array([[-13.21045, 1.2355469, 0.13355469],
               [0.0, -0.85659297, 0.2743407],
               [0.0, 0.11858518, 1.01185852]])
Ad_vk = [A0, A1, A2, A3, A4, A5, A6, A7]

outputKi= np.zeros([8,2,3])
outputP= np.zeros([8,3,3])

for i, element in enumerate(Ad_vk):
    # print(f"Indeks {i}: {element}")
    Ai = element
    Z = Ai@Y+Bd@Wi
    lmi = cp.vstack([
        cp.hstack([Y, Z.T ,Y, Wi.T]), #baris 1
        cp.hstack([Z, Y, np.zeros([3,3]), np.zeros([3,2])]), #baris 2
        cp.hstack([Y, np.zeros([3,3]), invQts, np.zeros([3,2])]), #baris 3
        cp.hstack([Wi, np.zeros([2,3]), np.zeros([2,3]), invRts]) #baris 4
        ])
    constraints = [lmi>=(0.3*np.eye(11)), Y>>0] # lmi definit positif dgn batasan lebih spesifik agar nilai Y dan Wi tidak nol
    obj = cp.Minimize(0)
    problem = cp.Problem(obj, constraints)
    problem.solve(solver=cp.SCS)

    if problem.status == cp.OPTIMAL:
        P = np.linalg.inv(Y.value)
        Ki = Wi.value@P
        outputP[i]=P
        outputKi[i]=Ki
    else:
        print("Problem not solved")
        print("Status:", problem.status)
print("Output Ki : ", outputKi.shape)
print("Output P : ", outputP.shape)


