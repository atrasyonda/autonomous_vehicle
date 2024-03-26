import cvxpy as cp
import numpy as np

# Definisikan matriks A, B
A = np.array([[1, 0.1], [0, 1]])
B = np.array([[0], [0.1]])

# Horizon prediksi
N = 20

# Inisialisasi variabel kontrol
delta_u = [cp.Variable((1, 1))for _ in range(N)]

# Inisialisasi variabel state awal
x0 = np.array([[0], [0]])  # Misal kondisi awal

# Inisialisasi variabel state
x = [cp.Variable((2, 1)) for _ in range(N + 1)]
x[0] = x0

# Inisialisasi fungsi biaya
J = 0

# Inisialisasi batasan untuk state dan input kontrol
X_constraint = cp.Parameter(2)
U_constraint = cp.Parameter(2)

# Masukkan batasan untuk state dan input kontrol
X_constraint.value = np.array([-100, 100])  # Misal batasan state [-100, 100]
U_constraint.value = np.array([-1, 1])  # Misal batasan input kontrol [-1, 1]


# Loop untuk menghitung fungsi biaya
for k in range(N):
    # Fungsi biaya lokal (misal: minimalkan norma Euclidean)
    J += cp.norm(x[k]) + cp.norm(delta_u[k])

    # Batasan dinamika sistem
    constraints = [x[k + 1] == A @ x[k] + B @ delta_u[k]]

    # Batasan state
    constraints += [cp.norm(x[k + 1]) <= X_constraint]

    # Batasan input kontrol
    constraints += [cp.norm(delta_u[k]) <= U_constraint]

    # Tambahkan batasan ke masalah optimisasi
    problem = cp.Problem(cp.Minimize(J), constraints)

    # Solver dengan GUROBI
    problem.solve(solver=cp.GUROBI)

    # Print hasil
    print("Optimal input control:", delta_u.value)
