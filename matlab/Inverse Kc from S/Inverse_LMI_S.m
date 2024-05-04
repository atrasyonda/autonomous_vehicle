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

S = [0.465 0 0;0 23.813 76.596;0 76596 257.251]
Z = inv(S)

B = [-0.1 1; 0 0; 0 -0.1];

u_max = [20;1.42];
% u_max = [1.42;20];
u_max_squared = u_max*u_max'


setlmis([])

[K1,n,sK1] = lmivar(2,[2 3]);
% [K2,n,sK2] = lmivar(2,[2 3]);
% [K3,n,sK3] = lmivar(2,[2 3]);
% [K4,n,sK4] = lmivar(2,[2 3]);
% [K5,n,sK5] = lmivar(2,[2 3]);
% [K6,n,sK6] = lmivar(2,[2 3]);
% [K7,n,sK7] = lmivar(2,[2 3]);
% [K8,n,sK8] = lmivar(2,[2 3]);

% Perumusan persamaan LMI -->  L(X) < R(X)
% +p : left side ; -p : right side , p adalah persamaan LMI ke-p
lmiterm([1 1 1 0],-Z) % Z [1,1]
lmiterm([1 2 1 0],A1*Z)
lmiterm([1 2 1 K1],B,Z) % (Ai +BKi)Z [2,1]
lmiterm([1 2 2 Z],-1,1) % Z [2,2]
% 
% lmiterm([2 1 1 Z],-1,1) % Z [1,1]
% lmiterm([2 2 1 0],A2*Z)
% lmiterm([2 2 1 K2],B*Z,1) % (Ai +BKi)Z [2,1]
% lmiterm([2 2 2 Z],-1,1) % Z [2,2]
% 
% lmiterm([3 1 1 Z],-1,1) % Z [1,1]
% lmiterm([3 2 1 0],A3*Z)
% lmiterm([3 2 1 K3],B*Z,1) % (Ai +BKi)Z [2,1]
% lmiterm([3 2 2 Z],-1,1) % Z [2,2]
% 
% lmiterm([4 1 1 Z],-1,1) % Z [1,1]
% lmiterm([4 2 1 0],A4*Z)
% lmiterm([4 2 1 K4],B*Z,1) % (Ai +BKi)Z [2,1]
% lmiterm([4 2 2 Z],-1,1) % Z [2,2]
% 
% lmiterm([5 1 1 Z],-1,1) % Z [1,1]
% lmiterm([5 2 1 0],A5*Z)
% lmiterm([5 2 1 K5],B*Z,1) % (Ai +BKi)Z [2,1]
% lmiterm([5 2 2 Z],-1,1) % Z [2,2]
% 
% lmiterm([6 1 1 Z],-1,1) % Z [1,1]
% lmiterm([6 2 1 0],A6*Z)
% lmiterm([6 2 1 K6],B*Z,1) % (Ai +BKi)Z [2,1]
% lmiterm([6 2 2 Z],-1,1) % Z [2,2]
% 
% lmiterm([7 1 1 Z],-1,1) % Z [1,1]
% lmiterm([7 2 1 0],A7*Z)
% lmiterm([7 2 1 K7],B*Z,1) % (Ai +BKi)Z [2,1]
% lmiterm([7 2 2 Z],-1,1) % Z [2,2]
% 
% lmiterm([8 1 1 Z],-1,1) % Z [1,1]
% lmiterm([8 2 1 0],A8*Z)
% lmiterm([8 2 1 K8],B*Z,1) % (Ai +BKi)Z [2,1]
% lmiterm([8 2 2 Z],-1,1) % Z [2,2]

%==================================================
% 
lmiterm([9 1 1 0],eye(2)) % I 
lmiterm([9 2 1 K1],1,1) 
lmiterm([9 2 2 0],-inv(Z)) % -KZK
lmiterm([-9 1 1 0],eye(2)-u_max_squared) 
% KZK -u < 0 --> -KZK > -u --> (I - KZK) > (I - u)

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

K1_value = dec2mat(LMIs, xfeas, K1)

evalsys = evallmi(LMIs,xfeas);

[lhs,rhs] = showlmi(evalsys,1);
eig(lhs);


% lmiinfo(LMIs)