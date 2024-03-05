###############################################################################
# This function simulates an open loop state-space model:
# x_{k+1} = A x_{k} + B u_{k}
# y_{k}   = C x_{k}
# starting from an initial condition x_{0}

# Input parameters:
# A,B,C - system matrices 
# U - the input matrix, its dimensions are \in \mathbb{R}^{m \times simSteps},  where m is the input vector dimension
# Output parameters:
# Y - simulated output - dimensions \in \mathbb{R}^{r \times simSteps}, where r is the output vector dimension
# X - simulated state - dimensions  \in \mathbb{R}^{n \times simSteps}, where n is the state vector dimension

def systemSimulate(A,B,C,U,x0):
    import numpy as np
    simTime=U.shape[1] # 200 -- number of time samples
    n=A.shape[0] # 4 (3) -- state dimension
    r=C.shape[1] # 1 (3) -- output dimension
    X=np.zeros(shape=(n,simTime+1)) # 4x201 (3x201)
    Y=np.zeros(shape=(r,simTime)) # 1x200 (3x200)
    for i in range(0,simTime): # 0 to 199
        if i==0:
            X[:,[i]]=x0 
            Y[:,[i]]=np.matmul(C,x0) # Y = C*X0
            X[:,[i+1]]=np.matmul(A,x0)+np.matmul(B,U[:,[i]]) # X1 = A*X0 + B*U0
        else:
            Y[:,[i]]=np.matmul(C,X[:,[i]]) # Y = C*Xk
            X[:,[i+1]]=np.matmul(A,X[:,[i]])+np.matmul(B,U[:,[i]]) # Xk+1 = A*Xk + B*Uk
    print('Y = ',Y)
    return Y,X

###############################################################################
# end of function
###############################################################################