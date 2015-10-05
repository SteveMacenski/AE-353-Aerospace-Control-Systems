%AE353 Problem 3 on HW 1

clear all

%knowns 
a1 = [0 9; -9 0];
a2 = [-1 3;-9 0];
xo = [1;-4];

%Finding eigenvalue vectors for scalar version 
[V1,D1] = eig(a1);
[V2,D2] = eig(a2);

%Diagonalize them to find A diag
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
t1 = linspace(0,2,100);
y5 = cos(9*t1)-(4)*sin(9*t1);
t2 = linspace(0,2,20);
y6 = exp(-.5.*t2).*[cos(sqrt(107)/2.*t2)-2.4168.*sin(sqrt(107)/2.*t2)]
%plotting
plot(t,y1,t,y2,t1,y5,'o',t2,y6,'o')
title('Steve Macenski Problem 2');
xlabel('time t');
legend('Y for U = 0 Matrix','Y for U=-[1 6]x Matrix','Y for U = 0 Scalar','Y for U = -[1 6]x Scalar');
