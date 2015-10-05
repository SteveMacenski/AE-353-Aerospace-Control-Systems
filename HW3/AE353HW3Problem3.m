clc; clear all;clf
%WIth a state space representation, find K, and Acl, then plot the step
%response and eigenvalues of the given Ks. 

%Part E of 3
A = [0 1 0;0 0 0;0 0 0];
B = [0;1/15;-1];
x0 = [0;.01;0];
K = [.015 1.65 0];
% 
T = 0:.1:600;
i = 0;
for i=1:length(T);
    x(i,:) = expm((A-B*K)*T(i))*x0;
end
figure(1)
plot(T,x)
legend('Theta','Theta Dot','V');

figure(2)
T2 = 0:.1:500;
A2 = [0 1 0;-2.42e-6 0 0;0 0 0];
K2 = [0.0152 1.0317 -.0413];
for i=1:length(T2);
    x(i,:) = expm((A2-B*K2)*T2(i))*x0;
end 
plot(T2,x);
legend('Theta','Theta Dot','V');


