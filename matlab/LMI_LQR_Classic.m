close all
clear
clc
%======= INI PERUMUSAN LMI UNTUK DYNAMIC CONTROLLER ======
A1 = [0.02 -0.87 -0.67; 0 -5.48 0.6; 0 0.74 2.64];
A2 = [0.02 -0.87 -0.65; 0 -5.48 0.6; 0 0.74 2.64];
A3 = [0.99 0 -0.01; 0 0.97 -0.2; 0 0 1.01];
A4 = [0.99 0 0.01; 0 0.97 -0.2; 0 0 1.01];
A5 = [0.02 0.87 0.65; 0 -5.48 0.6; 0 0.74 2.64];
A6 = [0.02 0.87 0.67; 0 -5.48 0.6; 0 0.74 2.64];
A7 = [0.99 0 -0.01; 0 0.97 -0.2; 0 0 1.01];
A8 = [0.99 0 0.01; 0 0.97 -0.2; 0 0 1.01];

B = [0 0.01; 0.35139092 0; 0.32431276 0];

C = [0 0 1];

Q = 0.9*diag([0.66 0.01 0.33]);
% Q = diag([10 15 10]);

R = 0.1*diag([0.5 0.5]);
% R = diag([4 1]);

[K1, S1, P1] = lqr(A1,B,Q,R);
[K2, S2, P2] = lqr(A2,B,Q,R);
[K3, S3, P3] = lqr(A3,B,Q,R);
[K4, S4, P4] = lqr(A4,B,Q,R);
[K5, S5, P5] = lqr(A5,B,Q,R);
[K6, S6, P6] = lqr(A6,B,Q,R);
[K7, S7, P7] = lqr(A7,B,Q,R);
[K8, S8, P8] = lqr(A8,B,Q,R);

K1
K2
K3
K4
K5
K6
K7
K8