%Steve Macenski AE 353 Problem 3 HW 7
clc;clf;clear all;
% poles

w = logspace(-3,3,100000);
s = w*j;

Hs = 2*(s.^2 + s + 8)./(4*s.^4 + 6*s.^3 + 49*s.^2 + 16*s + 64);

angles = angle(Hs);
mag = abs(Hs);

figure(1);
semilogx(w,radtodeg(angles));
title('Steve Macenski, angle plot P.2');
figure(2);
loglog(w,mag);
title('Steve Macenski, mag plot P.2');

