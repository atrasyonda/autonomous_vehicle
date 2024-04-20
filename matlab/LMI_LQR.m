close all
clear
clc

A = [1 -0.142 0;0.142 1 0.00999583; 0 0 1]; % Ac_pk[0]

B = [-0.1 1; 0 0; 0 -0.1];
C = [1 1 1];
Q = diag([1 1 3])
R = diag([1 3])

C_m = [inv(Q) zeros(3,2); zeros(2,3) inv(R)];

setlmis([])
[Y,n,sY] = lmivar(1,[3 1]);
[Wi,n,sWi] = lmivar(2,[2 3]);

% Perumusan persamaan LMI -->  L(X) < R(X)
% +p : left side ; -p : right side , p adalah persamaan LMI ke-p
lmiterm([-1 1 1 Y],1,1) % Y [1,1]
lmiterm([-1 2 1 Y],A,1)
lmiterm([-1 2 1 Wi],B,1) % AY + BWi [2,1]
lmiterm([-1 2 2 Y],1,1) % Y [2,2]
lmiterm([-1 3 1 Y],1,1) % Y [3,1]
lmiterm([-1 3 2 0],zeros(3)) % O 3x3 [3,2]
lmiterm([-1 3 3 0],inv(Q)) % inv(Q)
lmiterm([-1 4 1 Wi],1,1) % Wi [4,1]
lmiterm([-1 4 2 0],zeros(2,3)) % O 2x3 [4,2]
lmiterm([-1 4 3 0],zeros(2,3)) % O 2x3 [4,3]
lmiterm([-1 4 4 0],inv(R)) % inv(R)

% lmiterm([1 0 0 0],0)

LMIs = getlmis;

[tmin,xfeas] = feasp(LMIs);

W_value = dec2mat(LMIs,xfeas,Wi)
Y_value = dec2mat(LMIs, xfeas, Y)

evalsys = evallmi(LMIs,xfeas);
[lhs,rhs] = showlmi(evalsys,1)

EigenvalueY = eig(Y_value);

Ki = W_value*inv(Y_value);

% lmiinfo(LMIs)