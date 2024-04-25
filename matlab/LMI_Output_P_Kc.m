close all
clear
clc

%======= INI PERUMUSAN LMI UNTUK KINEMATIC CONTROLLER ======
A1 = [1 -0.142 0;0.142 1 0.00999583; 0 0 1]; % Ac_pk[0]
A2 = [1 -0.142 0; 0.142 1 0.00999583; 0 0 1];
A3 = [1 -0.142 0; 0.142 1 1.99916677; 0 0 1];
A4 = [1 -0.142 0; 0.142 1 1.99916677; 0 0 1];
A5 = [1 0.142 0; -0.142 1 0.00999583; 0 0 1];
A6 = [1 0.142 0; -0.142 1 0.00999583; 0 0 1];
A7 = [1 0.142 0; -0.142 1 1.99916677; 0 0 1];
A8 = [1 0.142 0; -0.142 1 1.99916677; 0 0 1];


B = [-0.1 1; 0 0; 0 -0.1];
C = [1 1 1];
Q = diag([1 1 3]);
R = diag([1 3]);

C_m = [inv(Q) zeros(3,2); zeros(2,3) inv(R)];

setlmis([])

[Y,n,sY] = lmivar(1,[3 1]);

[W1,n,sW1] = lmivar(2,[2 3]);
[W2,n,sW2] = lmivar(2,[2 3]);
[W3,n,sW3] = lmivar(2,[2 3]);
[W4,n,sW4] = lmivar(2,[2 3]);
[W5,n,sW5] = lmivar(2,[2 3]);
[W6,n,sW6] = lmivar(2,[2 3]);
[W7,n,sW7] = lmivar(2,[2 3]);
[W8,n,sW8] = lmivar(2,[2 3]);

% Perumusan persamaan LMI -->  L(X) < R(X)
% +p : left side ; -p : right side , p adalah persamaan LMI ke-p
lmiterm([-1 1 1 Y],1,1) % Y [1,1]
lmiterm([-1 2 1 Y],A1,1)
lmiterm([-1 2 1 W1],B,1) % AY + BWi [2,1]
lmiterm([-1 2 2 Y],1,1) % Y [2,2]
lmiterm([-1 3 1 Y],1,1) % Y [3,1]
lmiterm([-1 3 2 0],zeros(3)) % O 3x3 [3,2]
lmiterm([-1 3 3 0],inv(Q)) % inv(Q)
lmiterm([-1 4 1 W1],1,1) % Wi [4,1]
lmiterm([-1 4 2 0],zeros(2,3)) % O 2x3 [4,2]
lmiterm([-1 4 3 0],zeros(2,3)) % O 2x3 [4,3]
lmiterm([-1 4 4 0],inv(R)) % inv(R)

lmiterm([-2 1 1 Y],1,1) % Y [1,1]
lmiterm([-2 2 1 Y],A2,1)
lmiterm([-2 2 1 W2],B,1) % AY + BWi [2,1]
lmiterm([-2 2 2 Y],1,1) % Y [2,2]
lmiterm([-2 3 1 Y],1,1) % Y [3,1]
lmiterm([-2 3 2 0],zeros(3)) % O 3x3 [3,2]
lmiterm([-2 3 3 0],inv(Q)) % inv(Q)
lmiterm([-2 4 1 W2],1,1) % Wi [4,1]
lmiterm([-2 4 2 0],zeros(2,3)) % O 2x3 [4,2]
lmiterm([-2 4 3 0],zeros(2,3)) % O 2x3 [4,3]
lmiterm([-2 4 4 0],inv(R)) % inv(R)

lmiterm([-3 1 1 Y],1,1) % Y [1,1]
lmiterm([-3 2 1 Y],A3,1)
lmiterm([-3 2 1 W3],B,1) % AY + BWi [2,1]
lmiterm([-3 2 2 Y],1,1) % Y [2,2]
lmiterm([-3 3 1 Y],1,1) % Y [3,1]
lmiterm([-3 3 2 0],zeros(3)) % O 3x3 [3,2]
lmiterm([-3 3 3 0],inv(Q)) % inv(Q)
lmiterm([-3 4 1 W3],1,1) % Wi [4,1]
lmiterm([-3 4 2 0],zeros(2,3)) % O 2x3 [4,2]
lmiterm([-3 4 3 0],zeros(2,3)) % O 2x3 [4,3]
lmiterm([-3 4 4 0],inv(R)) % inv(R)

lmiterm([-4 1 1 Y],1,1) % Y [1,1]
lmiterm([-4 2 1 Y],A4,1)
lmiterm([-4 2 1 W4],B,1) % AY + BWi [2,1]
lmiterm([-4 2 2 Y],1,1) % Y [2,2]
lmiterm([-4 3 1 Y],1,1) % Y [3,1]
lmiterm([-4 3 2 0],zeros(3)) % O 3x3 [3,2]
lmiterm([-4 3 3 0],inv(Q)) % inv(Q)
lmiterm([-4 4 1 W4],1,1) % Wi [4,1]
lmiterm([-4 4 2 0],zeros(2,3)) % O 2x3 [4,2]
lmiterm([-4 4 3 0],zeros(2,3)) % O 2x3 [4,3]
lmiterm([-4 4 4 0],inv(R)) % inv(R)

lmiterm([-5 1 1 Y],1,1) % Y [1,1]
lmiterm([-5 2 1 Y],A5,1)
lmiterm([-5 2 1 W5],B,1) % AY + BWi [2,1]
lmiterm([-5 2 2 Y],1,1) % Y [2,2]
lmiterm([-5 3 1 Y],1,1) % Y [3,1]
lmiterm([-5 3 2 0],zeros(3)) % O 3x3 [3,2]
lmiterm([-5 3 3 0],inv(Q)) % inv(Q)
lmiterm([-5 4 1 W5],1,1) % Wi [4,1]
lmiterm([-5 4 2 0],zeros(2,3)) % O 2x3 [4,2]
lmiterm([-5 4 3 0],zeros(2,3)) % O 2x3 [4,3]
lmiterm([-5 4 4 0],inv(R)) % inv(R)

lmiterm([-6 1 1 Y],1,1) % Y [1,1]
lmiterm([-6 2 1 Y],A6,1)
lmiterm([-6 2 1 W6],B,1) % AY + BWi [2,1]
lmiterm([-6 2 2 Y],1,1) % Y [2,2]
lmiterm([-6 3 1 Y],1,1) % Y [3,1]
lmiterm([-6 3 2 0],zeros(3)) % O 3x3 [3,2]
lmiterm([-6 3 3 0],inv(Q)) % inv(Q)
lmiterm([-6 4 1 W6],1,1) % Wi [4,1]
lmiterm([-6 4 2 0],zeros(2,3)) % O 2x3 [4,2]
lmiterm([-6 4 3 0],zeros(2,3)) % O 2x3 [4,3]
lmiterm([-6 4 4 0],inv(R)) % inv(R)

lmiterm([-7 1 1 Y],1,1) % Y [1,1]
lmiterm([-7 2 1 Y],A7,1)
lmiterm([-7 2 1 W7],B,1) % AY + BWi [2,1]
lmiterm([-7 2 2 Y],1,1) % Y [2,2]
lmiterm([-7 3 1 Y],1,1) % Y [3,1]
lmiterm([-7 3 2 0],zeros(3)) % O 3x3 [3,2]
lmiterm([-7 3 3 0],inv(Q)) % inv(Q)
lmiterm([-7 4 1 W7],1,1) % Wi [4,1]
lmiterm([-7 4 2 0],zeros(2,3)) % O 2x3 [4,2]
lmiterm([-7 4 3 0],zeros(2,3)) % O 2x3 [4,3]
lmiterm([-7 4 4 0],inv(R)) % inv(R)

lmiterm([-8 1 1 Y],1,1) % Y [1,1]
lmiterm([-8 2 1 Y],A8,1)
lmiterm([-8 2 1 W8],B,1) % AY + BWi [2,1]
lmiterm([-8 2 2 Y],1,1) % Y [2,2]
lmiterm([-8 3 1 Y],1,1) % Y [3,1]
lmiterm([-8 3 2 0],zeros(3)) % O 3x3 [3,2]
lmiterm([-8 3 3 0],inv(Q)) % inv(Q)
lmiterm([-8 4 1 W8],1,1) % Wi [4,1]
lmiterm([-8 4 2 0],zeros(2,3)) % O 2x3 [4,2]
lmiterm([-8 4 3 0],zeros(2,3)) % O 2x3 [4,3]
lmiterm([-8 4 4 0],inv(R)) % inv(R)

LMIs = getlmis;

[tmin,xfeas] = feasp(LMIs);

W1_value = dec2mat(LMIs,xfeas,W1);
W2_value = dec2mat(LMIs,xfeas,W2);
W3_value = dec2mat(LMIs,xfeas,W3);
W4_value = dec2mat(LMIs,xfeas,W4);
W5_value = dec2mat(LMIs,xfeas,W5);
W6_value = dec2mat(LMIs,xfeas,W6);
W7_value = dec2mat(LMIs,xfeas,W7);
W8_value = dec2mat(LMIs,xfeas,W8);
Y_value = dec2mat(LMIs, xfeas, Y)

evalsys = evallmi(LMIs,xfeas);
EigenvalueY = eig(Y_value)

P = inv(Y_value)

K1 = W1_value*inv(Y_value);
K2 = W2_value*inv(Y_value);
K3 = W3_value*inv(Y_value);
K4 = W4_value*inv(Y_value);
K5 = W5_value*inv(Y_value);
K6 = W6_value*inv(Y_value);
K7 = W7_value*inv(Y_value);
K8 = W8_value*inv(Y_value);

% [lhs,rhs] = showlmi(evalsys,8)
% eig(rhs)
% lmiinfo(LMIs)

eigenvalue1 = eig(A1-B*K1)
eigenvalue2 = eig(A2-B*K2)
eigenvalue3 = eig(A3-B*K3)
eigenvalue4 = eig(A4-B*K4)
eigenvalue5 = eig(A5-B*K5)
eigenvalue6 = eig(A6-B*K6)
eigenvalue7 = eig(A7-B*K7)
eigenvalue8 = eig(A8-B*K8)
