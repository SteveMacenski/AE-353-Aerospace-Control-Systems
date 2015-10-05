%Steve Macenski AE 353 Problem 2 HW 7
clc;clear all;clf;

w = logspace(-3,3,10000);
s = w*j; 

Hs = .2./(s.^2 + .2*s);
angles = angle(Hs);
mag  = abs(Hs);

figure(1);
semilogx(w,radtodeg(angles));
title('Steve Macenski, angle plot P.2');
figure(2);
loglog(w,mag);
title('Steve Macenski, mag plot P.2');

a = [0 1;0 -.2];
b = [0;.2];
c = [1 0];
t = linspace(0,25,100);

s = .4253; %corresponding to H(w) mag = 1
i=0;
for i = 1:length(t);
    
    y(i) = c*([0;0] - (inv((s*eye(2)-a))*b))*sin(s.*t(i)) + (c*inv((s*eye(2) - a))*b).*sin(s.*t(i));

end
figure(3);
plot(t,y);