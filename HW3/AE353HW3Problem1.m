clf;clc;clear all

%WIth a state space representation, find K, and Acl, then plot the step
%response and eigenvalues of the given Ks. 


A = [0 1;0 -.2];
B = [0;.2];
C = [1 0];

%Part D
K1 = [303.4323 45.0520];
K2 = [249.195 31.188];
K3 = [226.3829 23.0794];

A1 = A-B*K1;
A2 = A-B*K2;
A3 = A-B*K3;
figure(1)
subplot(1,2,1);
    hold on
    step(ss(A1,B*K1(1),C,0));
    step(ss(A2,B*K2(1),C,0));
    step(ss(A3,B*K3(1),C,0));
    legend('M = .1','M = .2','M = .3');
subplot(1,2,2);
    hold on 
    Q=eig(A1);
    R=eig(A2);
    S=eig(A3);
    Eigval = [Q;R;S];
    plot(Eigval,'o')
    xlabel('Real Part');
    ylabel('Imaginary Part');
    title('Steve Macenski');
    grid on
    
%As M's increase, the peaks of the step response overshoot also increase.
%The calculated M are equivalent to the overshoot as the Mp formula
%predicts. The eigenvalues move less negative as M increases with the
%imaginary part remaining constant at 2*pi. 

%Part E
K4 = [303.4323 45.0520];
K5 = [33.711 14.4987];
K6 = [12.137 8.2103];

A4 = A-B*K4;
A5 = A-B*K5;
A6 = A-B*K6;
figure(2)
subplot(1,2,1);
    hold on
    step(ss(A4,B*K4(1),C,0));
    step(ss(A5,B*K5(1),C,0));
    step(ss(A6,B*K6(1),C,0));
    legend('T = .5','T = 1.5','T = 2.5');
subplot(1,2,2);
    hold on 
    T=eig(A4);
    U=eig(A5);
    V=eig(A6);
    Eigval = [T;U;V];
    plot(Eigval,'o')
    xlabel('Real Part');
    ylabel('Imaginary Part');
    title('Steve Macenski');
    grid on