%AE 353 hw 2 Problem 3
clc; clear all; clf

% computes the step response of a state-space control system 


% A = [-6 10;-9 0];
% B = [-10;0];
% C = [0 1];
% 
% step(ss(A,B,C,0))
% x = linspace(0,2,10);
% e1 = exp(-3+9i); e2 = exp(-3-9i);
% hold on
% plot(x,1-(1/3).*exp(-3*x).*(sin(9.*x)+3.*cos(9.*x)),'o');

%PART B

% a = [-6 10; -9 0];
% b = [1;0];
% c = [0 1];
% 
% step(ss(a,b,c,0))
% hold on 
% x = linspace(0,2,10);
% plot(x,-.1+(1/30).*exp(-3*x).*(sin(9.*x)+3.*cos(9.*x)),'o');

%PART C
a1 = [-6 10 1;-9 0 0;0 1 0];
b1 = [5;0;0];
c1 = [0 1 0];
step(ss(a1,b1,c1,0));
hold on 
x = linspace(0,2,10);



