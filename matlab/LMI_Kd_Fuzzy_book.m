close all
clear
clc
%======= INI PERUMUSAN LMI UNTUK DYNAMIC CONTROLLER ======
% INI MATRIKS A YANG SUDAH DIBENARKAN  VX [0.1,20]
A1 = [0.019999404, -0.869355054, -0.668971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
A2 = [0.019999404, -0.869355054, -0.648971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
A3 = [0.994980803, -0.00434677527, -0.0132948557; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
A4 = [0.994980803, -0.00434677527, 0.00670514434; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
A5 = [0.019999404, 0.869355054, 0.648971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
A6 = [0.019999404, 0.869355054, 0.668971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
A7 = [0.994980803, 0.00434677527, -0.00670514434; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
A8 = [0.994980803, 0.00434677527, 0.0132948557; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];

% INI MATRIKS A BENAR & VX [0.1,10] Delta [-0.25,0.25]
% A1 = [0.019999404, -0.869355054, -0.668971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
% A2 = [0.019999404, -0.869355054, -0.648971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
% A3 = [0.990140401, -0.00869355054, -0.0165897113; 0, 0.935206591, -0.0939538137; 0, 0.00736183054, 0.936000163];
% A4 = [0.990140401, -0.00869355054, 0.00341028869; 0, 0.935206591, -0.0939538137; 0, 0.00736183054, 0.936000163];
% A5 = [0.019999404, 0.869355054, 0.648971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
% A6 = [0.019999404, 0.869355054, 0.668971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
% A7 = [0.990140401, 0.00869355054, -0.00341028869; 0, 0.935206591, -0.0939538137; 0, 0.00736183054, 0.936000163];
% A8 = [0.990140401, 0.00869355054, 0.0165897113; 0, 0.935206591, -0.0939538137; 0, 0.00736183054, 0.936000163];

% INI MATRIKS A BENAR & VX [0.1,10] Delta [-0.8,0.8]
% A1 = [0.0199994, -2.52072418, -1.92070893; 0, -4.5228347, 1.3286503; 0, 1.61898091, -4.73082295];
% A2 = [0.0199994, -2.52072418, -1.90070893; 0, -4.5228347, 1.3286503; 0, 1.61898091, -4.73082295];
% A3 = [0.9901404, -0.02520724, -0.02910709; 0, 0.94477165, -0.0867035; 0, 0.01618981, 0.94269177];
% A4 = [0.9901404, -0.02520724, -0.00910709; 0, 0.94477165, -0.0867035; 0, 0.01618981, 0.94269177];
% A5 = [0.0199994, 2.52072418, 1.90070893; 0, -4.5228347, 1.3286503; 0, 1.61898091, -4.73082295];
% A6 = [0.0199994, 2.52072418, 1.92070893; 0, -4.5228347, 1.3286503; 0, 1.61898091, -4.73082295];
% A7 = [0.9901404, 0.02520724, 0.00910709; 0, 0.94477165, -0.0867035; 0, 0.01618981, 0.94269177];
% A8 = [0.9901404, 0.02520724, 0.02910709; 0, 0.94477165, -0.0867035; 0, 0.01618981, 0.94269177];

B = [0 0.01; 0.35139092 0; 0.32431276 0];
C = [1 0 0;0 1 0;0 0 1];

Q = 0.9*diag([0.66 0.01 0.33]);
R = 0.1*diag([0.5 0.5]);

% Q = diag([10 15 10]);
% R = diag([4 1])

% Q = diag([1 1 1]);
% R = diag([1 1]);


setlmis([])

[Y,n,sY] = lmivar(1,[3 1]);

% [W1,n,sW1] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
% [W2,n,sW2] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
% [W3,n,sW3] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
% [W4,n,sW4] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
% [W5,n,sW5] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
% [W6,n,sW6] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
% [W7,n,sW7] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
% [W8,n,sW8] = lmivar(3,[0 n+1 n+2;n+3 0 0]);

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

% INI UNTUK LMI < 0
lmiterm([-1 1 1 Y],1,1) % Y > 0
% lmiterm([1 1 1 0],0.5*eye(3))

lmiterm([2 1 1 Y],1,A1,'1')
lmiterm([2 1 1 W1],-B,1,'1')
lmiterm([2 1 1 Y],2,1)
lmiterm([2 2 1 Y],1,1) % +2Y
lmiterm([2 2 2 0],-inv(Q))
lmiterm([2 3 1 W1],-1,1)
lmiterm([2 3 2 0],zeros(2,3))
lmiterm([2 3 3 0],-inv(R))

lmiterm([3 1 1 Y],1,A2,'2')
lmiterm([3 1 1 W2],-B,1,'2')
lmiterm([3 1 1 Y],1,1) % +2Y
lmiterm([3 2 1 Y],1,1)
lmiterm([3 2 2 0],-inv(Q))
lmiterm([3 3 1 W2],-1,1)
lmiterm([3 3 2 0],zeros(2,3))
lmiterm([3 3 3 0],-inv(R))

lmiterm([4 1 1 Y],1,A3,'3')
lmiterm([4 1 1 W3],-B,1,'3')
lmiterm([4 1 1 Y],1,1) % +2Y
lmiterm([4 2 1 Y],1,1)
lmiterm([4 2 2 0],-inv(Q))
lmiterm([4 3 1 W3],-1,1)
lmiterm([4 3 2 0],zeros(2,3))
lmiterm([4 3 3 0],-inv(R))

lmiterm([5 1 1 Y],1,A4,'4')
lmiterm([5 1 1 W4],-B,1,'4')
lmiterm([5 2 1 Y],1,1)
lmiterm([5 1 1 Y],1,1) % +2Y
lmiterm([5 2 2 0],-inv(Q))
lmiterm([5 3 1 W4],-1,1)
lmiterm([5 3 2 0],zeros(2,3))
lmiterm([5 3 3 0],-inv(R))

lmiterm([6 1 1 Y],1,A5,'5')
lmiterm([6 1 1 W5],-B,1,'5')
lmiterm([6 1 1 Y],1,1) % +2Y
lmiterm([6 2 1 Y],1,1)
lmiterm([6 2 2 0],-inv(Q))
lmiterm([6 3 1 W5],-1,1)
lmiterm([6 3 2 0],zeros(2,3))
lmiterm([6 3 3 0],-inv(R))

lmiterm([7 1 1 Y],1,A6,'6')
lmiterm([7 1 1 W6],-B,1,'6')
lmiterm([7 1 1 Y],1,1) % +2Y
lmiterm([7 2 1 Y],1,1)
lmiterm([7 2 2 0],-inv(Q))
lmiterm([7 3 1 W6],-1,1)
lmiterm([7 3 2 0],zeros(2,3))
lmiterm([7 3 3 0],-inv(R))

lmiterm([8 1 1 Y],1,A7,'7')
lmiterm([8 1 1 W7],-B,1,'7')
lmiterm([8 1 1 Y],1,1) % +2Y
lmiterm([8 2 1 Y],1,1)
lmiterm([8 2 2 0],-inv(Q))
lmiterm([8 3 1 W7],-1,1)
lmiterm([8 3 2 0],zeros(2,3))
lmiterm([8 3 3 0],-inv(R))

lmiterm([9 1 1 Y],1,A8,'8')
lmiterm([9 1 1 W8],-B,1,'8')
lmiterm([9 1 1 Y],1,1) % +2Y
lmiterm([9 2 1 Y],1,1)
lmiterm([9 2 2 0],-inv(Q))
lmiterm([9 3 1 W8],-1,1)
lmiterm([9 3 2 0],zeros(2,3))
lmiterm([9 3 3 0],-inv(R))

LMIs = getlmis;

option = zeros(1,5); % default value for all parameters 
target = -1
[tmin,xfeas] = feasp(LMIs,option,target);
tmin

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

K1 = W1_value*P
K2 = W2_value*P
K3 = W3_value*P
K4 = W4_value*P
K5 = W5_value*P
K6 = W6_value*P
K7 = W7_value*P
K8 = W8_value*P

close_loop_eigenvalue1 = eig(A1-B*K1)
close_loop_eigenvalue2 = eig(A2-B*K2)
close_loop_eigenvalue3 = eig(A3-B*K3)
close_loop_eigenvalue4 = eig(A4-B*K4)
close_loop_eigenvalue5 = eig(A5-B*K5)
close_loop_eigenvalue6 = eig(A6-B*K6)
close_loop_eigenvalue7 = eig(A7-B*K7)
close_loop_eigenvalue8 = eig(A8-B*K8)

% [lhs,rhs] = showlmi(evalsys,2)
% eig(lhs)
% lmiinfo(LMIs)

% ================== SIMULATION =========================
% x0 = [20;-1;0.9]; % Error[Vx, Vy, Omega]
% t = 0:0.005:10;
% sys1 = ss(A3-B*K3,B,C,0);
% [y,t,x] = initial(sys1,x0,t);
% error_xdot = y(:,1);
% error_ydot = y(:,2);
% error_psidot = y(:,3);
% % 
% plot(t,error_xdot,"r",t,error_ydot,"green", t,error_psidot,"blue");
