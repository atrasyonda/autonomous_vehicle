import numpy as np
from function import Kinematic
# from constants import *



Ad = np.array([[ 0.93066667,  0.         , -0.31866667,  0.        ],
               [ 0.          , 1.          ,  0.02       ,  0.        ],
               [ 0.04066667,  0.         ,  0.75133333,  0.        ],
               [ 0.02       ,  0.4        ,  0.         ,  1.        ]])

Bd = np.array([[0.50666667],
               [0.        ],
               [0.50666667],
               [0.        ]])

Cd = np.array([[0, 1, 0, 0],
               [0, 0, 0, 1]])

Dd = 0
# print("State Space Matrices")
# print("Ad: ", Ad)
# print("Bd: ", Bd)
# print("Cd: ", Cd)
# print("Dd: ", Dd)

def mpc_simplification(Ad, Bd, Cd, Dd, hz):
    '''This function creates the compact matrices for Model Predictive Control'''
    # db - double bar
    # dbt - double bar transpose
    # dc - double circumflex
    """
    A_aug = np.array([[A , B],
                      [O , I]])
    state ---> np.array([[x_k+1] = np.array([[x k],
                        [u_k])              [u_k-1]])
    B_aug = np.array([[B],
                      [I]])
    input ---> np.array([[u k] = np.array([[delta_u_k]])
    """

    A_aug=np.concatenate((Ad,Bd),axis=1)
    # print("A_aug: ", A_aug)
    temp1=np.zeros((np.size(Bd,1),np.size(Ad,1)))
    temp2=np.identity(np.size(Bd,1))
    temp=np.concatenate((temp1,temp2),axis=1)
    # print("temp: ", temp)
    A_aug=np.concatenate((A_aug,temp),axis=0)
    print("A_aug: ", A_aug)
    


    B_aug=np.concatenate((Bd,np.identity(np.size(Bd,1))),axis=0)
    # print("B_aug: ", B_aug)

    C_aug=np.concatenate((Cd,np.zeros((np.size(Cd,0),np.size(Bd,1)))),axis=1)
    D_aug=Dd

    Q=np.matrix('1 0;0 1') # weights for outputs (all samples, except the last one)
    S=np.matrix('1 0;0 1') # weights for the final horizon period outputs
    R=np.matrix('1') # weights for inputs (only 1 input in our case)


    CQC=np.matmul(np.transpose(C_aug),Q)
    CQC=np.matmul(CQC,C_aug)

    CSC=np.matmul(np.transpose(C_aug),S)
    CSC=np.matmul(CSC,C_aug)

    QC=np.matmul(Q,C_aug)
    SC=np.matmul(S,C_aug)


    Qdb=np.zeros((np.size(CQC,0)*hz,np.size(CQC,1)*hz))
    Tdb=np.zeros((np.size(QC,0)*hz,np.size(QC,1)*hz))
    Rdb=np.zeros((np.size(R,0)*hz,np.size(R,1)*hz))
    Cdb=np.zeros((np.size(B_aug,0)*hz,np.size(B_aug,1)*hz))
    Adc=np.zeros((np.size(A_aug,0)*hz,np.size(A_aug,1)))

    for i in range(0,hz):
        if i == hz-1:
            Qdb[np.size(CSC,0)*i:np.size(CSC,0)*i+CSC.shape[0],np.size(CSC,1)*i:np.size(CSC,1)*i+CSC.shape[1]]=CSC
            Tdb[np.size(SC,0)*i:np.size(SC,0)*i+SC.shape[0],np.size(SC,1)*i:np.size(SC,1)*i+SC.shape[1]]=SC
        else:
            Qdb[np.size(CQC,0)*i:np.size(CQC,0)*i+CQC.shape[0],np.size(CQC,1)*i:np.size(CQC,1)*i+CQC.shape[1]]=CQC
            Tdb[np.size(QC,0)*i:np.size(QC,0)*i+QC.shape[0],np.size(QC,1)*i:np.size(QC,1)*i+QC.shape[1]]=QC

        Rdb[np.size(R,0)*i:np.size(R,0)*i+R.shape[0],np.size(R,1)*i:np.size(R,1)*i+R.shape[1]]=R

        for j in range(0,hz):
            if j<=i:
                Cdb[np.size(B_aug,0)*i:np.size(B_aug,0)*i+B_aug.shape[0],np.size(B_aug,1)*j:np.size(B_aug,1)*j+B_aug.shape[1]]=np.matmul(np.linalg.matrix_power(A_aug,((i+1)-(j+1))),B_aug)

        Adc[np.size(A_aug,0)*i:np.size(A_aug,0)*i+A_aug.shape[0],0:0+A_aug.shape[1]]=np.linalg.matrix_power(A_aug,i+1)

    Hdb=np.matmul(np.transpose(Cdb),Qdb)
    Hdb=np.matmul(Hdb,Cdb)+Rdb

    temp=np.matmul(np.transpose(Adc),Qdb)
    temp=np.matmul(temp,Cdb)

    temp2=np.matmul(-Tdb,Cdb)
    Fdbt=np.concatenate((temp,temp2),axis=0)

    return Hdb,Fdbt,Cdb,Adc


N=20
Tc=0.1
t=np.arange(0,10+Tc,Tc) 

Hdb,Fdbt,Cdb,Adc = mpc_simplification(Ad, Bd, Cd, Dd, N)



"""""

def trajectory_generator():
    '''This method creates the trajectory for a car to follow'''
    # time from 0 to 10 seconds, sample time (Tc=0.1 second)
    r=4 # amplitudo sinusoidal trajectory
    f= 0.01 # frequency
    
    x_dot=20 # car velocity constant
    lane_width=7

    # Define the x length, depends on the car's longitudinal velocity
    x=np.linspace(0,x_dot*t[-1],num=len(t))

    # Define trajectories
    aaa=-28/100**2
    aaa=aaa/1.1
    if aaa<0:
        bbb=14
    else:
        bbb=-14
    y_1=aaa*(x+lane_width-100)**2+bbb
    y_2=2*r*np.sin(2*np.pi*f*x)
    y=(y_1+y_2)/2

    # Vector of x and y changes per sample time
    dx=x[1:len(x)]-x[0:len(x)-1]
    dy=y[1:len(y)]-y[0:len(y)-1]

    # Define the reference yaw angles
    psi=np.zeros(len(x))
    psiInt=psi
    psi[0]=np.arctan2(dy[0],dx[0])
    psi[1:len(psi)]=np.arctan2(dy[0:len(dy)],dx[0:len(dx)])

    # We want the yaw angle to keep track the amount of rotations
    dpsi=psi[1:len(psi)]-psi[0:len(psi)-1]
    psiInt[0]=psi[0]
    for i in range(1,len(psiInt)):
        if dpsi[i-1]<-np.pi:
            psiInt[i]=psiInt[i-1]+(dpsi[i-1]+2*np.pi)
        elif dpsi[i-1]>np.pi:
            psiInt[i]=psiInt[i-1]+(dpsi[i-1]-2*np.pi)
        else:
            psiInt[i]=psiInt[i-1]+dpsi[i-1]
    return psiInt,x,y

outputs = 3 # x,y, psi
# Generate the refence signals
psi_ref,X_ref,Y_ref=trajectory_generator()
sim_length=len(t) # Number of control loop iterations
refSignals=np.zeros(len(X_ref)*outputs)

# Build up the reference signal vector:
# refSignal = [psi_ref_0, Y_ref_0, psi_ref_1, Y_ref_1, psi_ref_2, Y_ref_2, ... etc.]
k=0
for i in range(0,len(refSignals),outputs):
    refSignals[i]=psi_ref[k]
    refSignals[i+1]=Y_ref[k]
    k=k+1


# Load the initial states
# If you want to put numbers here, please make sure that they are float and not
# integers. It means that you should add a point there.
# Example: Please write 0. in stead of 0 (Please add the point to make it float)
y_dot=0.
psi=0.
psi_dot=0.
Y=Y_ref[0]+10.

states=np.array([y_dot,psi,psi_dot,Y])
statesTotal=np.zeros((len(t),len(states))) # It will keep track of all your states during the entire manoeuvre
statesTotal[0][0:len(states)]=states
psi_opt_total=np.zeros((len(t),N))
Y_opt_total=np.zeros((len(t),N))

# Load the initial input
U1=0 # Input at t = -1 s (steering wheel angle in rad (delta))
UTotal=np.zeros(len(t)) # To keep track all your inputs over time
UTotal[0]=U1

"""