%AE353HW1PROB2
clear all 
%Defining matrix A's and initial condiditions 
a1 = [0 1; 0 -5];
a2 = [0 1; -50 -15];
xo = [0;4];

%Finding the eigenvalues, V = matrix with eigenvalues on diagonal  
[V1,D1] = eig(a1);
[V2,D2] = eig(a2);
V1
%Diagonalize A by V-1AV = Adiag
diagA1 = inv(V1)*a1*V1 
diagA2 = inv(V2)*a2*V2 


% % Finding the exponential function using for loop 
t = linspace(0,2,1e3);
for i = 1:length(t)
    x1(:,i) = expm(a1*t(i))*xo;
    x2(:,i) = expm(a2*t(i))*xo;
    
    %and for the diagonalized version
    x3(:,i) = V1*expm(diagA1*t(i))*inv(V1)*xo;
    x4(:,i) = V2*expm(diagA2*t(i))*inv(V2)*xo;
end

%Multiplying X result to find Y 
y1 = [1 0]*x1;
y2 = [1 0]*x2;
y3 = [1 0]*x3;
y4 = [1 0]*x4;
t1 = linspace(0,2,25)
y5 = 1/5*(4-4.*exp(-5*t1));
y6 = 4/5*(exp(-5*t1)+-exp(-10*t1));
%plotting
plot(t,y1,t,y2,t1,y5,'o',t1,y6,'o')
title('Steve Macenski Problem 2');
xlabel('time t');
legend('Y for U = 0 Matrix','Y for U=-[5 1]x Matrix','Y for U = 0 Scalar','Y for U = -[5 1] Scalar');