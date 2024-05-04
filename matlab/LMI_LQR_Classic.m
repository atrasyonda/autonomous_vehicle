close all
clear
clc
%======= INI PERUMUSAN LMI UNTUK DYNAMIC CONTROLLER ======
A1 = [0.019999404, -0.869355054, -0.668971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
A2 = [0.019999404, -0.869355054, -0.648971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
A3 = [0.994980803, -0.00434677527, -0.0132948557; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
A4 = [0.994980803, -0.00434677527, 0.00670514434; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
A5 = [0.019999404, 0.869355054, 0.648971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
A6 = [0.019999404, 0.869355054, 0.668971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
A7 = [0.994980803, 0.00434677527, -0.00670514434; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
A8 = [0.994980803, 0.00434677527, 0.0132948557; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];


B = [0 0.01; 0.35139092 0; 0.32431276 0];

C = [1 0 0;0 0 0;0 0 1];

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

% P1
% eigS1 = eig(S1)
% P2
% eigS2 = eig(S2)
% P3
% eigS3 = eig(S3)
% P4
% eigS4 = eig(S4)

% Coba disimulasikan
% x0 = [0.1;0;0]; % [Vx, Vy, Omega]
x0 = [20;0;0.9]; % Error[Vx, Vy, Omega]
t = 0:0.005:10;

sys1 = ss(A1-B*K1,B,C,0);
Kdc = dcgain(sys1);
% Kr = [1/Kdc(1) ; 1/Kdc(2)]
% sys1_scaled = ss(A1-B*K1,B*Kr,C,0);
[y,t,x] = initial(sys1,x0,t);
% step(sys1);
% step(sys1_scaled);
error_xdot = y(:,1);
error_psidot = y(:,3);

% plot(t,error_xdot,"r",t,error_psidot,"b")