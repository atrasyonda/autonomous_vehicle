import numpy as np
from scipy.optimize import minimize

# Fungsi cost Jk
def cost_function(u, x, Q, R, P, N):
    cost = 0
    for i in range(N):
        cost += np.dot(np.dot(x[:, i].T, Q), x[:, i]) + np.dot(u[i].T, np.dot(R, u[i]))
    cost += np.dot(np.dot(x[:, N].T, P), x[:, N])
    return cost

# Batasan yang diberikan
def constraints(u, x, A, B, N, x0, umin, umax):
    X = np.zeros((len(x0), N+1))
    X[:, 0] = x0
    for i in range(N):
        X[:, i+1] = np.dot(A, X[:, i]) + np.dot(B, u[i])
    return np.concatenate((X[:, 1:] - x, u - umin, umax - u))

# Inisialisasi parameter
N = 10  # Horizon time
n = 2   # State dimension
m = 1   # Control input dimension

# Matriks cost
Q = np.eye(n)
R = np.eye(m)
P = np.eye(n)

# Matriks sistem dinamis
A = np.eye(n)
B = np.eye(n)

# Inisialisasi kondisi awal
x0 = np.array([1, 2])

# Inisialisasi batasan kontrol
umin = -1
umax = 1

# Persyaratan batasan
cons = ({'type': 'ineq', 'fun': constraints, 'args': (A, B, N, x0, umin, umax)})

# Pencarian optimum menggunakan metode SLSQP
u_init = np.zeros((m, N))
res = minimize(cost_function, u_init, args=(x0, Q, R, P, N), constraints=cons, method='SLSQP')

# Solusi kontrol yang dihasilkan
u_opt = res.x

print("Solusi optimal untuk kontrol:", u_opt)
