close all
clear
clc
%======= INI PERUMUSAN LMI UNTUK DYNAMIC CONTROLLER ======

% INI MATRIKS A DENGAN delta[-0.25,0.25] Vx[0.1,20] Vy[-1,1]
% A1 = [0.02 -0.87 -0.67; 0 -5.48 0.6; 0 0.74 2.64];
% A2 = [0.02 -0.87 -0.65; 0 -5.48 0.6; 0 0.74 2.64];
% A3 = [0.99 0 -0.01; 0 0.97 -0.2; 0 0 1.01];
% A4 = [0.99 0 0.01; 0 0.97 -0.2; 0 0 1.01];
% A5 = [0.02 0.87 0.65; 0 -5.48 0.6; 0 0.74 2.64];
% A6 = [0.02 0.87 0.67; 0 -5.48 0.6; 0 0.74 2.64];
% A7 = [0.99 0 -0.01; 0 0.97 -0.2; 0 0 1.01];
% A8 = [0.99 0 0.01; 0 0.97 -0.2; 0 0 1.01];

% INI MATRIKS A YANG SUDAH DIBENARKAN  VX [0.1,20]
A1 = [0.019999404, -0.869355054, -0.668971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
A2 = [0.019999404, -0.869355054, -0.648971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
A3 = [0.994980803, -0.00434677527, -0.0132948557; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
A4 = [0.994980803, -0.00434677527, 0.00670514434; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
A5 = [0.019999404, 0.869355054, 0.648971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
A6 = [0.019999404, 0.869355054, 0.668971131; 0, -5.47934087, 0.603618627; 0, 0.736183054, -5.39998373];
A7 = [0.994980803, 0.00434677527, -0.00670514434; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
A8 = [0.994980803, 0.00434677527, 0.0132948557; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];

% INI MATRIKS A BENAR & VX [0.01,20]
% A1 = [-8.80000006, -8.69355054, -6.59971131; 0, -63.7934087, 6.04608627; 0, 7.36183054, -62.9998373];
% A2 = [-8.80000006, -8.69355054, -6.57971131; 0, -63.7934087, 6.04608627; 0, 7.36183054, -62.9998373];
% A3 = [0.994980803, -0.00434677527, -0.0132948557; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
% A4 = [0.994980803, -0.00434677527, 0.00670514434; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
% A5 = [-8.80000006, 8.69355054, 6.57971131; 0, -63.7934087, 6.04608627; 0, 7.36183054, -62.9998373];
% A6 = [-8.80000006, 8.69355054, 6.59971131; 0, -63.7934087, 6.04608627; 0, 7.36183054, -62.9998373];
% A7 = [0.994980803, 0.00434677527, -0.00670514434; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
% A8 = [0.994980803, 0.00434677527, 0.0132948557; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];

% INI MATRIKS A BENAR & VX [-0.1,20]
% A1 = [1.98000060, 0.869355054, 0.648971131; 0, 7.47934087, -0.603618627; 0, -0.736183054, 7.39998373];
% A2 = [1.98000060, 0.869355054, 0.668971131; 0, 7.47934087, -0.603618627; 0, -0.736183054, 7.39998373];
% A3 = [0.994980803, -0.00434677527, -0.0132948557; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
% A4 = [0.994980803, -0.00434677527, 0.00670514434; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
% A5 = [1.98000060, -0.869355054, -0.668971131; 0, 7.47934087, -0.603618627; 0, -0.736183054, 7.39998373];
% A6 = [1.98000060, -0.869355054, -0.648971131; 0, 7.47934087, -0.603618627; 0, -0.736183054, 7.39998373];
% A7 = [0.994980803, 0.00434677527, -0.00670514434; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];
% A8 = [0.994980803, 0.00434677527, 0.0132948557; 0, 0.967603296, -0.196976907; 0, 0.00368091527, 0.968000081];

B = [0 0.01; 0.35139092 0; 0.32431276 0];

C = [1 0 0;0 0 0;0 0 1];

Q = 0.9*diag([0.66 0.01 0.33]);
% Q = diag([10 15 10]);

R = 0.1*diag([0.5 0.5]);
% R = diag([4 1])

setlmis([])

[Y,n,sY] = lmivar(1,[3 1]);

[W1,n,sW1] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
[W2,n,sW2] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
[W3,n,sW3] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
[W4,n,sW4] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
[W5,n,sW5] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
[W6,n,sW6] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
[W7,n,sW7] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
[W8,n,sW8] = lmivar(3,[0 n+1 n+2;n+3 0 0]);

% [W1,n,sW1] = lmivar(2,[2 3]);
% [W2,n,sW2] = lmivar(2,[2 3]);
% [W3,n,sW3] = lmivar(2,[2 3]);
% [W4,n,sW4] = lmivar(2,[2 3]);
% [W5,n,sW5] = lmivar(2,[2 3]);
% [W6,n,sW6] = lmivar(2,[2 3]);
% [W7,n,sW7] = lmivar(2,[2 3]);
% [W8,n,sW8] = lmivar(2,[2 3]);

% Perumusan persamaan LMI -->  L(X) < R(X)
% +p : left side ; -p : right side , p adalah persamaan LMI ke-p

% INI UNTUK LMI > 0
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

% INI UNTUK LMI < 0
% lmiterm([1 1 1 Y],1,1) % Y [1,1]
% lmiterm([1 2 1 Y],A1,1)
% lmiterm([1 2 1 W1],B,1) % AY + BWi [2,1]
% lmiterm([1 2 2 Y],1,1) % Y [2,2]
% lmiterm([1 3 1 Y],1,1) % Y [3,1]
% lmiterm([1 3 2 0],zeros(3)) % O 3x3 [3,2]
% lmiterm([1 3 3 0],inv(Q)) % inv(Q)
% lmiterm([1 4 1 W1],1,1) % Wi [4,1]
% lmiterm([1 4 2 0],zeros(2,3)) % O 2x3 [4,2]
% lmiterm([1 4 3 0],zeros(2,3)) % O 2x3 [4,3]
% lmiterm([1 4 4 0],inv(R)) % inv(R)
% 
% lmiterm([2 1 1 Y],1,1) % Y [1,1]
% lmiterm([2 2 1 Y],A2,1)
% lmiterm([2 2 1 W2],B,1) % AY + BWi [2,1]
% lmiterm([2 2 2 Y],1,1) % Y [2,2]
% lmiterm([2 3 1 Y],1,1) % Y [3,1]
% lmiterm([2 3 2 0],zeros(3)) % O 3x3 [3,2]
% lmiterm([2 3 3 0],inv(Q)) % inv(Q)
% lmiterm([2 4 1 W2],1,1) % Wi [4,1]
% lmiterm([2 4 2 0],zeros(2,3)) % O 2x3 [4,2]
% lmiterm([2 4 3 0],zeros(2,3)) % O 2x3 [4,3]
% lmiterm([2 4 4 0],inv(R)) % inv(R)
% 
% lmiterm([3 1 1 Y],1,1) % Y [1,1]
% lmiterm([3 2 1 Y],A3,1)
% lmiterm([3 2 1 W3],B,1) % AY + BWi [2,1]
% lmiterm([3 2 2 Y],1,1) % Y [2,2]
% lmiterm([3 3 1 Y],1,1) % Y [3,1]
% lmiterm([3 3 2 0],zeros(3)) % O 3x3 [3,2]
% lmiterm([3 3 3 0],inv(Q)) % inv(Q)
% lmiterm([3 4 1 W3],1,1) % Wi [4,1]
% lmiterm([3 4 2 0],zeros(2,3)) % O 2x3 [4,2]
% lmiterm([3 4 3 0],zeros(2,3)) % O 2x3 [4,3]
% lmiterm([3 4 4 0],inv(R)) % inv(R)
% 
% lmiterm([4 1 1 Y],1,1) % Y [1,1]
% lmiterm([4 2 1 Y],A4,1)
% lmiterm([4 2 1 W4],B,1) % AY + BWi [2,1]
% lmiterm([4 2 2 Y],1,1) % Y [2,2]
% lmiterm([4 3 1 Y],1,1) % Y [3,1]
% lmiterm([4 3 2 0],zeros(3)) % O 3x3 [3,2]
% lmiterm([4 3 3 0],inv(Q)) % inv(Q)
% lmiterm([4 4 1 W4],1,1) % Wi [4,1]
% lmiterm([4 4 2 0],zeros(2,3)) % O 2x3 [4,2]
% lmiterm([4 4 3 0],zeros(2,3)) % O 2x3 [4,3]
% lmiterm([4 4 4 0],inv(R)) % inv(R)
% 
% lmiterm([5 1 1 Y],1,1) % Y [1,1]
% lmiterm([5 2 1 Y],A5,1)
% lmiterm([5 2 1 W5],B,1) % AY + BWi [2,1]
% lmiterm([5 2 2 Y],1,1) % Y [2,2]
% lmiterm([5 3 1 Y],1,1) % Y [3,1]
% lmiterm([5 3 2 0],zeros(3)) % O 3x3 [3,2]
% lmiterm([5 3 3 0],inv(Q)) % inv(Q)
% lmiterm([5 4 1 W5],1,1) % Wi [4,1]
% lmiterm([5 4 2 0],zeros(2,3)) % O 2x3 [4,2]
% lmiterm([5 4 3 0],zeros(2,3)) % O 2x3 [4,3]
% lmiterm([5 4 4 0],inv(R)) % inv(R)
% 
% lmiterm([6 1 1 Y],1,1) % Y [1,1]
% lmiterm([6 2 1 Y],A6,1)
% lmiterm([6 2 1 W6],B,1) % AY + BWi [2,1]
% lmiterm([6 2 2 Y],1,1) % Y [2,2]
% lmiterm([6 3 1 Y],1,1) % Y [3,1]
% lmiterm([6 3 2 0],zeros(3)) % O 3x3 [3,2]
% lmiterm([6 3 3 0],inv(Q)) % inv(Q)
% lmiterm([6 4 1 W6],1,1) % Wi [4,1]
% lmiterm([6 4 2 0],zeros(2,3)) % O 2x3 [4,2]
% lmiterm([6 4 3 0],zeros(2,3)) % O 2x3 [4,3]
% lmiterm([6 4 4 0],inv(R)) % inv(R)
% 
% lmiterm([7 1 1 Y],1,1) % Y [1,1]
% lmiterm([7 2 1 Y],A7,1)
% lmiterm([7 2 1 W7],B,1) % AY + BWi [2,1]
% lmiterm([7 2 2 Y],1,1) % Y [2,2]
% lmiterm([7 3 1 Y],1,1) % Y [3,1]
% lmiterm([7 3 2 0],zeros(3)) % O 3x3 [3,2]
% lmiterm([7 3 3 0],inv(Q)) % inv(Q)
% lmiterm([7 4 1 W7],1,1) % Wi [4,1]
% lmiterm([7 4 2 0],zeros(2,3)) % O 2x3 [4,2]
% lmiterm([7 4 3 0],zeros(2,3)) % O 2x3 [4,3]
% lmiterm([7 4 4 0],inv(R)) % inv(R)
% 
% lmiterm([8 1 1 Y],1,1) % Y [1,1]
% lmiterm([8 2 1 Y],A8,1)
% lmiterm([8 2 1 W8],B,1) % AY + BWi [2,1]
% lmiterm([8 2 2 Y],1,1) % Y [2,2]
% lmiterm([8 3 1 Y],1,1) % Y [3,1]
% lmiterm([8 3 2 0],zeros(3)) % O 3x3 [3,2]
% lmiterm([8 3 3 0],inv(Q)) % inv(Q)
% lmiterm([8 4 1 W8],1,1) % Wi [4,1]
% lmiterm([8 4 2 0],zeros(2,3)) % O 2x3 [4,2]
% lmiterm([8 4 3 0],zeros(2,3)) % O 2x3 [4,3]
% lmiterm([8 4 4 0],inv(R)) % inv(R)

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

P = inv(Y_value);

K1 = W1_value*inv(Y_value)
K2 = W2_value*inv(Y_value)
K3 = W3_value*inv(Y_value)
K4 = W4_value*inv(Y_value)
K5 = W5_value*inv(Y_value)
K6 = W6_value*inv(Y_value)
K7 = W7_value*inv(Y_value)
K8 = W8_value*inv(Y_value)

% K1
close_loop_eigenvalue1 = eig(A1-B*K1)
close_loop_eigenvalue2 = eig(A2-B*K2)
close_loop_eigenvalue3 = eig(A3-B*K3)
close_loop_eigenvalue4 = eig(A4-B*K4)
close_loop_eigenvalue5 = eig(A5-B*K5)
close_loop_eigenvalue6 = eig(A6-B*K6)
close_loop_eigenvalue7 = eig(A7-B*K7)
close_loop_eigenvalue8 = eig(A8-B*K8)

% [lhs,rhs] = showlmi(evalsys,1)
% eig(lhs)
% lmiinfo(LMIs)

% Coba disimulasikan
x0 = [20;0;0.9]; % Error[Vx, Vy, Omega]
t = 0:0.005:10;
sys1 = ss(A1-B*K1,B,C,0);
[y,t,x] = initial(sys1,x0,t);
error_xdot = y(:,1);
error_psidot = y(:,3);

plot(t,error_xdot,"r",t,error_psidot,"b")