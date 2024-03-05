import numpy as np
import support_file_car as sfc

kinematic=sfc.kinematic()
dynamic=sfc.dynamic()
lpv=sfc.LPV()

a=5

b=a**3
# print(b)

# print("==== Ini adalah Matrix Kinematic")
# [Ak,Bk]=kinematic.state_space()
# print(Ak)
# print(Bk)
# print("==== Ini adalah Matrix Dynamic")
# [Ad,Bd]=dynamic.state_space()
# print(Ad)
# print(Bd)
# print("==== Ini adalah Matrix LPV Kinematik")
# Ac = lpv.getKinematic()
# print(Ac)
# print("==== Ini adalah Matrix LPV Dinamik")
Ad = lpv.getDynamic()
print(Ad)


