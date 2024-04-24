close all
clear
clc

A8 = [1 -0.142 0;0.142 1 0.00999583; 0 0 1]; % Ac_pk[0]
A7 = [1 -0.142 0; 0.142 1 0.00999583; 0 0 1];
A6 = [1 -0.142 0; 0.142 1 1.99916677; 0 0 1];
A5 = [1 -0.142 0; 0.142 1 1.99916677; 0 0 1];
A4 = [1 0.142 0; -0.142 1 0.00999583; 0 0 1];
A3 = [1 0.142 0; -0.142 1 0.00999583; 0 0 1];
A2 = [1 0.142 0; -0.142 1 1.99916677; 0 0 1];
A1 = [1 0.142 0; -0.142 1 1.99916677; 0 0 1];

K8 = [9.6373 18.2116 71.4823; -0.0013 2.0348 7.4270];
K7 = [9.6373 18.2116 71.4823; -0.0013 2.0348 7.4270];
K6 = [9.7145 18.1337 108.5361; 0.0066 2.0273 11.2804];
K5 = [9.7145 18.1337 108.5361; 0.0066 2.0273 11.2804];
K4 = [4.0455 20.2546 71.6308; -0.5821 1.9633 7.4437];
K3 = [4.0455 20.2546 71.6308; -0.5821 1.9633 7.4437];
K2 = [4.1183 20.2086 108.7616; -0.5746 1.9585 11.3031];
K1 = [4.1183 20.2086 108.7616; -0.5746 1.9585 11.3031];

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
% lmiterm([9 1 1 Z],K1,K1') % 
% lmiterm([9 1 1 0],-u_max_squared) % Ki*ZKi' - u_max^2 <0
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

[lhs,rhs] = showlmi(evalsys,1);
eig(lhs);


u_max_squared
u1_squared = K1*Z_value*K1'

% lmiinfo(LMIs)