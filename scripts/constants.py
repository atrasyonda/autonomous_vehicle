#!/usr/bin/env python3
import numpy as np

n = 3  # number of states
m = 2  # number of inputs


Tc=0.1      # time sampling for kinematic control 
Td=0.01     # time sampling for dynamic control 
# car constants
mass = 5       # massa mobil
lf = 0.1    # panjang antara CoM dg roda depan
lr = 0.1    # panjang antara CoM dg roda belakang
I = 20      # momen inersia
Caf = 19000 # koefisien kekakuan roda depan
Car = 33000 # koefisien kekakuan roda belakang

rho = 1.184 # densitas udara (kg/m^3)
Cd = 20     # koefisien drag
Af = 30     # front sectional area of the vehicle
miu = 0.5   # koefisien gesekan
g = 9.8     # konstanta gravitasi

# schedulling vector variable
psi_dot_min= -1.42  # batas bawah kecepatan anguler (rad/s)
psi_dot_max= 1.42   # batas atas kecepatan anguler (rad/s)
xr_dot_min= 0.1     # batas bawah reference kecepatan linier (m/s)
xr_dot_max= 20      # batas atas reference kecepatan linier (m/s)
psi_min= -0.05      # batas bawah sudut orientasi (rad)
psi_max= 0.05       # batas atas sudut orientasi (rad)

delta_min = -15  # batas bawah sudut steering (deg) --> referensi -0.25 rad
delta_max = 15   # batas atas sudut steering (deg) --> referensi 0.25 rad
x_dot_min = 0.1     # batas bawah kecepatan longitudinal / linier (m/s)
x_dot_max = 20      # batas atas kecepatan longitudinal / linier (m/s)
y_dot_min = -1      # batas bawah kecepatan lateral (m/s)
y_dot_max = 1       # batas atas kecepatan lateral (m/s)


# inisiasi state
psi_dot = 0
xr_dot = 0
psi = 0

x_dot = 0
y_dot = 0
delta = 0


Q_k = np.diag([1,1,3])  # kinematic control state weight matrix --> JURNAL
R_k = np.diag([1,3])  # kinematic control input weight matrix --> JURNAL

Q_d = 0.9*np.diag([0.66, 0.01, 0.33])  # dynamic control state weight matrix --> JURNAL
R_d = 0.1*np.diag([0.5, 0.5])  # dynamic control input weight matrix --> JURNAL