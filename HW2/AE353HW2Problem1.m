clc
clear all
clf

%this code plots the step response of scalar control system inputs

% %PART A
A = [-2];
B = [2];
C = [1];
figure(1)
step(ss(A,B,C,0));
hold on 
x = 0:.1:4.5
x1=linspace(0,4.5,1e5);
plot(x,-exp(-2*x)+1,'o',x1,.1,'r--',x1,.9,'r--',x1,.98,'g--',x1,1.02,'g--')
grid on
hold off 
% 
% %PART B
a = [-2];
b = [.5];
c = [1];
figure(2)
step(ss(a,b,c,0));
hold on
x = 0:.1:4.5;
plot(x,-.25*exp(-2*x)+.25,'o');
hold off

% %PART C
a1 = [-2 -17;1 0];
b1 = [.5;0];
c1 = [1 0];
figure(3)
step(ss(a1,b1,c1,0));
hold on
x = 0:.1:6;
plot(x,.125.*exp(-x).*sin(4*x),'o');