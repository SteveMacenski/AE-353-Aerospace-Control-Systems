clf;clc;clear all

%WIth a state space representation, find K, and Acl, then plot the step
%response and eigenvalues of the given Ks. 

A = [0 9;-9 0];
B = [1;0];
C = [0 1];

%Part D
K1 = [2 80/9];
K2 = [6 76/9];
K3 = [11 71/9];

A1 = A-B*K1;
A2 = A-B*K2;
A3 = A-B*K3;
figure(1)
x = 0:.1:12;
    hold on
    step(ss(A1,B*(-1/9),C,0));
    step(ss(A2,B*(-5/9),C,0));
    step(ss(A3,B*(-10/9),C,0));
    plot(x,1-exp(-x),'-o');
    legend('-1 -1','-1 -5','-1 -10','Exp function');