import numpy as np
import random


Tc=0.1
psi_dot_min= -1.42
psi_dot_max= 1.42
xr_dot_min= 0.1
xr_dot_max= 20
psi_min= -0.05
psi_max= 0.05

psi_dot = round(random.uniform(psi_dot_min, psi_dot_max),2)
xr_dot = round(random.uniform(xr_dot_min, xr_dot_max),2)
psi = round(random.uniform(psi_min, psi_max),2)
# print (psi_dot,xr_dot,psi)
# print ("psi dot = ", psi_dot)

nu0_psidot= (psi_dot_max-psi_dot)/(psi_dot_max-psi_dot_min)
nu1_psidot= 1-nu0_psidot
# print("nu0_psi dot = ", nu0_psidot)
# print("nu1_psi dot = ", nu1_psidot)

nu0_xrdot= (xr_dot_max-xr_dot)/(xr_dot_max-xr_dot_min)
nu1_xrdot= 1-nu0_xrdot
# print(nu0_xrdot)
# print(nu1_xrdot)
nu0_psi= (psi_max-psi)/(psi_max-psi_min)
nu1_psi= 1-nu0_psi
# print(nu0_psi)
# print(nu1_psi)


A0=np.array([
  [1, psi_dot_min*Tc, 0],
  [-psi_dot_min*Tc, 1, xr_dot_min*np.sin(psi_min)*Tc/psi_min],
  [0, 0, 1]
])
A1=np.array([
  [1, psi_dot_min*Tc, 0],
  [-psi_dot_min*Tc, 1, xr_dot_min*np.sin(psi_max)*Tc/psi_max],
  [0, 0, 1]
])
A2=np.array([
  [1, psi_dot_min*Tc, 0],
  [-psi_dot_min*Tc, 1, xr_dot_max*np.sin(psi_min)*Tc/psi_min],
  [0, 0, 1]
])
A3=np.array([
  [1, psi_dot_min*Tc, 0],
  [-psi_dot_min*Tc, 1, xr_dot_max*np.sin(psi_max)*Tc/psi_max],
  [0, 0, 1]
])
A4=np.array([
  [1, psi_dot_max*Tc, 0],
  [-psi_dot_max*Tc, 1, xr_dot_min*np.sin(psi_min)*Tc/psi_min],
  [0, 0, 1]
])
A5=np.array([
  [1, psi_dot_max*Tc, 0],
  [-psi_dot_max*Tc, 1, xr_dot_min*np.sin(psi_max)*Tc/psi_max],
  [0, 0, 1]
])
A6=np.array([
  [1, psi_dot_max*Tc, 0],
  [-psi_dot_max*Tc, 1, xr_dot_max*np.sin(psi_min)*Tc/psi_min],
  [0, 0, 1]
])
A7=np.array([
  [1, psi_dot_max*Tc, 0],
  [-psi_dot_max*Tc, 1, xr_dot_max*np.sin(psi_max)*Tc/psi_max],
  [0, 0, 1]
])
Ac_pk = [A0, A1, A2, A3, A4, A5, A6, A7]

miu = [
    [nu0_psidot*nu0_xrdot*nu0_psi],
    [nu0_psidot*nu0_xrdot*nu1_psi],
    [nu0_psidot*nu1_xrdot*nu0_psi],
    [nu0_psidot*nu1_xrdot*nu1_psi],
    [nu1_psidot*nu0_xrdot*nu0_psi],
    [nu1_psidot*nu0_xrdot*nu1_psi],
    [nu1_psidot*nu1_xrdot*nu0_psi],
    [nu1_psidot*nu1_xrdot*nu1_psi],
]
Ac=0
for i in range(8):
    # print ("VALUE = " , Ac_pk[i]*miu[i], "\n")
    Ac+=(Ac_pk[i]*miu[i])

print (Ac)



"""
# nc= 3 ,  psi_dot, xr_dot, psi
# miu_i(p(k)) = miu1, miu2, ... , miu ke 2^nc
# miu ke i = xi1 . xi2 . xij sampai xi nc (nc=3, j=3)
# xij (nu0 , nu1)
# nu0 = (pj max - pj) / (pj max - pj min)

 == Keterangan A0 - A7 ==
A0 = LLL
A1 = LLH
A2 = LHL
A3 = LHH
A4 = HLL
A5 = HLH
A6 = HHL
A7 = HHH

# rule = [
#     [0,0,0], #LLL
#     [0,0,1], #LLH
#     [0,1,0], #
#     [0,1,1],
#     [1,0,0],
#     [1,0,1],
#     [1,1,0],
#     [1,1,1]
# ]

# xi = [
#     [nu0_psidot, nu0_xrdot, nu0_psi],
#     [nu0_psidot, nu0_xrdot, nu1_psi],
#     [nu0_psidot, nu1_xrdot, nu0_psi],
#     [nu0_psidot, nu1_xrdot, nu1_psi],
#     [nu1_psidot, nu0_xrdot, nu0_psi],
#     [nu1_psidot, nu0_xrdot, nu1_psi],
#     [nu1_psidot, nu1_xrdot, nu0_psi],
#     [nu1_psidot, nu1_xrdot, nu1_psi],
# ]

# print(miu)
"""