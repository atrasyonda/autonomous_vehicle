import cvxpy as cp
import numpy as np

# menambahkan baris

n = 3  # number of states
m = 2  # number of inputs
# Ai=np.random.randint(1,5,(3,3)) #state matrix --> Jumlah Ai nanti ada 8 (sesuai Ac_pk A0 - A7)
B = np.array([[-0.1,0],[0,0],[0,-0.1]]) # input matrix
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

outputKi=[]

for i, element in enumerate(Ac_pk):
    # print(f"Indeks {i}: {element}")
    Ai = element
    print ("====== ITERASI KE -", i+1 , " ======")
    Z = Ai@Y+B@Wi
    lmi = cp.vstack([
        cp.hstack([Y, Z.T ,Y, Wi.T]), #baris 1
        cp.hstack([Z, Y, np.zeros([3,3]), np.zeros([3,2])]), #baris 2
        cp.hstack([Y, np.zeros([3,3]), invQts, np.zeros([3,2])]), #baris 3
        cp.hstack([Wi, np.zeros([2,3]), np.zeros([2,3]), invRts]) #baris 4
        ])
    constraints = [lmi>=(0.3*np.eye(11)), Y>>0] # lmi definit positif dgn batasan lebih spesifik agar nilai Y dan Wi tidak nol
    obj = cp.Minimize(0)
    problem = cp.Problem(obj, constraints)
    print("Solving problem")
    problem.solve(solver=cp.SCS)

    if problem.status == cp.OPTIMAL:
        print("Optimal value: ", problem.value)
        print("Y = ", Y.value)
        print("Wi = " , Wi.value)
        Ki = Wi.value@np.linalg.inv(Y.value)
        print("Ki = ", Ki)
        outputKi.append(Ki)
    else:
        print("Problem not solved")
        print("Status:", problem.status)
print ("Output Ki : ", outputKi)


