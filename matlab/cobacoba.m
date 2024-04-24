clear
clc

setlmis([]);
[Y,n,sY] = lmivar(1,[3 1]);

[W1,n,sW1] = lmivar(3,[0 n+1 n+2;n+3 0 0]);
[W2,n,sW2] = lmivar(3,[0 n+1 n+2;n+3 0 0]);

sY
sW1
sW2