close all
clear
clc

A_pk = cell(1,8);
A_pk{1} = [1 -0.142 0; 0.142 1 0.00999583; 0 0 1];
A_pk{2} = [1 -0.142 0; 0.142 1 0.00999583; 0 0 1];
A_pk{3} = [1 -0.142 0; 0.142 1 1.99916677; 0 0 1];
A_pk{4} = [1 -0.142 0; 0.142 1 1.99916677; 0 0 1];
A_pk{5} = [1 0.142 0; -0.142 1 0.00999583; 0 0 1];
A_pk{6} = [1 0.142 0; -0.142 1 0.00999583; 0 0 1];
A_pk{7} = [1 0.142 0; -0.142 1 1.99916677; 0 0 1];
A_pk{8} = [1 0.142 0; -0.142 1 1.99916677; 0 0 1]

W_pk = cell(1,8);
Ki = cell(1,8);

B = [-0.1 1; 0 0; 0 -0.1];
C = [1 1 1];
Q = diag([1 1 3])
R = diag([1 3])

% C_m = [inv(Q) zeros(3,2); zeros(2,3) inv(R)];

setlmis([])
[Y,n,sY] = lmivar(1,[3 1]);
[Wi,n,sWi] = lmivar(2,[2 3]);

% [B_m, n, sB_m] = lmivar(3,[sY,zeros(3);sWi,zeros(2,3)]);
for i = 1:8
    lmiterm([-1 1 1 Y],1,1) % Y [1,1]
    lmiterm([-1 2 1 Y],A_pk{i},1)
    lmiterm([-1 2 1 Wi],B,1) % AY + BWi [2,1]
    lmiterm([-1 2 2 Y],1,1) % Y [2,2]
    lmiterm([-1 3 1 Y],1,1) % Y [3,1]
    lmiterm([-1 3 2 0],zeros(3)) % O 3x3 [3,2]
    lmiterm([-1 3 3 0],inv(Q)) % inv(Q)
    lmiterm([-1 4 1 Wi],1,1) % Wi [4,1]
    lmiterm([-1 4 2 0],zeros(2,3)) % O 2x3 [4,2]
    lmiterm([-1 4 3 0],zeros(2,3)) % O 2x3 [4,3]
    lmiterm([-1 4 4 0],inv(R)) % inv(R)
    
    lmiterm([1 0 0 0],0)

    % Hitung LMI
    LMIs = getlmis;
    % Cari Solusi
    [~,xfeas] = feasp(LMIs);
    % Dapatkan hasil Wi
    W_value = dec2mat(LMIs,xfeas,Wi)
    W_pk{i} = W_value
end

Y_value = dec2mat(LMIs, xfeas, Y)
EigenvalueY = eig(Y_value);
for i = 1:8
    Ki{i} = W_value*inv(Y_value)
end