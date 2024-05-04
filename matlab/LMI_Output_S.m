close all
clear
clc

A1 = [1 -0.142 0;0.142 1 0.00999583; 0 0 1]; % Ac_pk[0]
A2 = [1 -0.142 0; 0.142 1 0.00999583; 0 0 1];
A3 = [1 -0.142 0; 0.142 1 1.99916677; 0 0 1];
A4 = [1 -0.142 0; 0.142 1 1.99916677; 0 0 1];
A5 = [1 0.142 0; -0.142 1 0.00999583; 0 0 1];
A6 = [1 0.142 0; -0.142 1 0.00999583; 0 0 1];
A7 = [1 0.142 0; -0.142 1 1.99916677; 0 0 1];
A8 = [1 0.142 0; -0.142 1 1.99916677; 0 0 1];

% K1 = [3.9546 -1.1006 -8.7579; -0.0602 0.1977 0.9053];
% K2 = [3.9546 -1.1006 -8.7579; -0.0602 0.1977 0.9053];
% K3 = [11.9189 -3.3173 -26.3958; -0.1530 0.3234 2.3083];
% K4 = [11.9189 -3.3173 -26.3958; -0.1530 0.3234 2.3083];
% K5 = [4.0707 -1.1330 -9.0151; -0.0739 0.0577 1.1197];
% K6 = [4.0707 -1.1330 -9.0151; -0.0739 0.0577 1.1197];
% K7 = [9.6357 -2.6818 -21.3395; -0.1216 0.0403 1.8436];
% K8 = [9.6357 -2.6818 -21.3395; -0.1216 0.0403 1.8436];

K1 = [30.8414, 26.5805, 110.3733; 1.7895, 2.6253, 10.3719];
K2 = [30.8414, 26.5805, 110.3733; 1.7895, 2.6253, 10.3719];
K3 = [39.8110, 32.1030, 171.4695; 2.7222, 3.2000, 16.7249];
K4 = [39.8110, 32.1030, 171.4695; 2.7222, 3.2000, 16.7249];
K5 = [22.9955, 26.7684, 103.3755; 0.9728, 2.4344, 9.9042];
K6 = [22.9955, 26.7684, 103.3755; 0.9728, 2.4344, 9.9042];
K7 = [31.9774, 32.3251, 164.5651; 1.9063, 3.0119, 16.2643];
K8 = [31.9774, 32.3251, 164.5651; 1.9063, 3.0119, 16.2643];

B = [-0.1 1; 0 0; 0 -0.1];

u_max = [20;1.42];
% u_max = [1.42;20];
u_max_squared = u_max*u_max'

setlmis([])

[Z,n,sZ] = lmivar(1,[3 1]);

% Perumusan persamaan LMI -->  L(X) < R(X)
% +p : left side ; -p : right side , p adalah persamaan LMI ke-p
lmiterm([1 1 1 Z],-1,1) % Z [1,1]
lmiterm([1 2 1 Z],(A1+B*K1),1) % (Ai +BKi)Z [2,1]
lmiterm([1 2 2 Z],-1,1) % Z [2,2]

lmiterm([2 1 1 Z],-1,1) % Z [1,1]
lmiterm([2 2 1 Z],(A2+B*K2),1) % (Ai +BKi)Z [2,1]
lmiterm([2 2 2 Z],-1,1) % Z [2,2]

lmiterm([3 1 1 Z],-1,1) % Z [1,1]
lmiterm([3 2 1 Z],(A3+B*K3),1) % (Ai +BKi)Z [2,1]
lmiterm([3 2 2 Z],-1,1) % Z [2,2]

lmiterm([4 1 1 Z],-1,1) % Z [1,1]
lmiterm([4 2 1 Z],(A4+B*K4),1) % (Ai +BKi)Z [2,1]
lmiterm([4 2 2 Z],-1,1) % Z [2,2]

lmiterm([5 1 1 Z],-1,1) % Z [1,1]
lmiterm([5 2 1 Z],(A5+B*K5),1) % (Ai +BKi)Z [2,1]
lmiterm([5 2 2 Z],-1,1) % Z [2,2]

lmiterm([6 1 1 Z],-1,1) % Z [1,1]
lmiterm([6 2 1 Z],(A6+B*K6),1) % (Ai +BKi)Z [2,1]
lmiterm([6 2 2 Z],-1,1) % Z [2,2]

lmiterm([7 1 1 Z],-1,1) % Z [1,1]
lmiterm([7 2 1 Z],(A7+B*K7),1) % (Ai +BKi)Z [2,1]
lmiterm([7 2 2 Z],-1,1) % Z [2,2]

lmiterm([8 1 1 Z],-1,1) % Z [1,1]
lmiterm([8 2 1 Z],(A8+B*K8),1) % (Ai +BKi)Z [2,1]
lmiterm([8 2 2 Z],-1,1) % Z [2,2]

%==================================================
% 
lmiterm([9 1 1 Z],K1,K1') % 
lmiterm([9 1 1 0],-u_max_squared) % Ki*ZKi' - u_max^2 <0
% 
% lmiterm([10 1 1 Z],K2,K2') % 
% lmiterm([10 1 1 0],-u_max_squared) % Ki*ZKi' - u_max^2 <0
% 
% lmiterm([11 1 1 Z],K3,K3') % 
% lmiterm([11 1 1 0],-u_max_squared) % Ki*ZKi' - u_max^2 <0
% 
% lmiterm([12 1 1 Z],K4,K4') % 
% lmiterm([12 1 1 0],-u_max_squared) % Ki*ZKi' - u_max^2 <0
% 
% lmiterm([13 1 1 Z],K5,K5') % 
% lmiterm([13 1 1 0],-u_max_squared) % Ki*ZKi' - u_max^2 <0
% 
% lmiterm([14 1 1 Z],K6,K6') % 
% lmiterm([14 1 1 0],-u_max_squared) % Ki*ZKi' - u_max^2 <0
% 
% lmiterm([15 1 1 Z],K7,K7') % 
% lmiterm([15 1 1 0],-u_max_squared) % Ki*ZKi' - u_max^2 <0
% 
% lmiterm([16 1 1 Z],K8,K8') % 
% lmiterm([16 1 1 0],-u_max_squared) % Ki*ZKi' - u_max^2 <0


LMIs = getlmis;

[tmin,xfeas] = feasp(LMIs);

Z_value = dec2mat(LMIs, xfeas, Z)

evalsys = evallmi(LMIs,xfeas);

S = inv(Z_value)

[lhs,rhs] = showlmi(evalsys,1)
eig(lhs)


% u_max_squared
% u1_squared = K1*Z_value*K1'
% u2_squared = K2*Z_value*K2'
% u3_squared = K3*Z_value*K3'
% u4_squared = K4*Z_value*K4'
% lmiinfo(LMIs)