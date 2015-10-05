
clc; clear all; clf
% computes the step response of a state-space control system 

% %PART A
% A = [0 1; -50 -15];
% B = [0;50];
% C = [1 0];
% figure(1)
% step(ss(A,B,C,0));
% hold on
% x = linspace(0,1.4,1e1);
% %plot(x,50*(exp(-5*x)-exp(-10*x))+1,'o');
% 
% e1 = exp(-5*x); e2 = exp(-10*x);
% plot(x,e2-2*e1+1,'o',0:.01:1.4,.1,'r-',0:.01:1.4,.9,'-r',0:.01:1.4,.98,'-g');

% % PART B
% a = [0 1; -50 -15];
% b = [0;10];
% c = [1 0];
% 
% 
% step(ss(a,b,c,0));
% hold on
% t = linspace(0,1.4,10);
% e1 = exp(-5.*t);
% e2 = exp(-10.*t);
% plot(t,(e2-2*e1+1)/5,'o',0:.01:1.4,.1,'r-',0:.01:1.4,.9,'-r',0:.01:1.4,.98,'-g');

%PART C

a1 = [0 1 0;-50 -15 -100; 1 0 0]
b1 = [0;10;0]
c1 = [0 0 1]

step(ss(a1,b1,c1,0));






