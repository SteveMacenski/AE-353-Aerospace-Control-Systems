function hw3prob04

% %%
%
% YOUR CODE HERE TO DEFINE...

% - initial condition
x0 = [0;0;0;0];

% - goal
xgoal = [1;0;4;0];

% - length of time interval
h = 1;

% - continuous-time state space system
A = [0 1 0 0;-8 -1 4 1/2; 0 0 0 1;4 .5 -4 -.5];
B = [0;0;0;1/2];
C = [1 0 0 0;0 0 1 0];

% - inputs (a column vector of length n=4)
udiscrete = ...
   [   39.8612;
   15.0541;
   21.3140;
   35.4525];

%
% %%


% SIMULATE THE SYSTEM AND ANIMATE THE RESULTS
Simulate(A,B,C,h,udiscrete,x0,xgoal)

function Simulate(A,B,C,h,udiscrete,x0,xgoal)

n = length(udiscrete);
tgoal = n*h;
ygoal = C*xgoal;
sys = ss(A,B,C,0);
trate = 1000;
ndt = trate+1;
dt = linspace(0,1,ndt);
dt = dt(2:end);
u = udiscrete(1);
t = 0;
for i=1:n
    u = [u repmat(udiscrete(n-(i-1)),1,ndt-1)];
    t = [t t(end)+dt];
end
for i=1:2*n
    u = [u zeros(1,ndt-1)];
    t = [t t(end)+dt];
end
y = lsim(sys,u,t,x0)';

M = CreateMass(y(:,1),ygoal,tgoal,max(t),0);
drawnow;

i = 1;
dt = max(t)/(length(t)-1);
pause(1);
tic;
while (i<length(t))
    if (toc > dt)
        tic;
        i = i+ceil(trate/50);
        M = SetMass(M,y(:,1:i),t(1:i),u(i));
        drawnow;
    end
end

function M = CreateMass(y,ygoal,tgoal,tmax,f)
clf;
subplot(2,1,1);
axis equal;
hold on;
axis([0 18 -1 1]);
set(gcf,'renderer','opengl');
linewidth = 2;
color = 'k';
M.L = 1.5;
M.w = 1;
M.r = 0.1;
M.d = 0.2;
M.peq = [M.L; M.L+M.w+M.L];
M.p = M.peq+y;
M.pgoal = M.peq+ygoal;
for i=1:2
    if (i==1)
        pbase = 0;
    else
        pbase = M.p(i-1)+M.w;
    end
    M.goal(i)=patch(M.pgoal(i)+M.w*[0 1 1 0 0],-(M.w/2)+M.w*[0 0 1 1 0],'g','linestyle','none','facecolor','g');
    M.mass(i)=line(M.p(i)+M.w*[0 1 1 0 0],-(M.w/2)+M.w*[0 0 1 1 0],'linewidth',linewidth,'color',color);
    M.spring(i)=patch(pbase+(M.p(i)-pbase)*[0 1 1 0 0],M.d+M.r*[-1 -1 1 1 -1],'b','linewidth',linewidth/2,'facecolor',1*[0 0 1],'edgecolor',color,'facealpha',0.5);
    M.damper(i)=patch(pbase+(M.p(i)-pbase)*[0 1 1 0 0],-M.d+M.r*[-1 -1 1 1 -1],'r','linewidth',linewidth/2,'facecolor',1*[1 0 0],'edgecolor',color,'facealpha',0.5);
end
M.fscale=0.1;
M.force=plot(M.p(2)+(M.w/2)+(f*M.fscale)*[0 1],[0 0],'m','linewidth',4);
M.title=title(sprintf('t = %5.2f',0));
subplot(2,1,2);
axis([0 tmax -15 15]);
hold on;
M.hplot(1) = plot(0,y(1),'k-','linewidth',2);
M.hplot(2) = plot(0,y(2),'k--','linewidth',2);
M.hgoal(1) = plot(tgoal,ygoal(1),'g.','linestyle','none','markersize',24);
M.hgoal(2) = plot(tgoal,ygoal(2),'g.','linestyle','none','markersize',24);
legend('x1','x2');
title('STATE RESPONSE');
xlabel('time');

function M = SetMass(M,y,t,f)
M.p = M.peq+y(:,end);
for i=1:2
    if (i==1)
        pbase = 0;
    else
        pbase = M.p(i-1)+M.w;
    end
    set(M.mass(i),'xdata',M.p(i)+M.w*[0 1 1 0 0]);
    set(M.spring(i),'xdata',pbase+(M.p(i)-pbase)*[0 1 1 0 0],'facealpha',1-sigmoid((M.p(i)-pbase)-M.L));
    set(M.damper(i),'xdata',pbase+(M.p(i)-pbase)*[0 1 1 0 0],'facealpha',1-sigmoid((M.p(i)-pbase)-M.L));
end
set(M.hplot(1),'xdata',t,'ydata',y(1,:));
set(M.hplot(2),'xdata',t,'ydata',y(2,:));
set(M.force,'xdata',M.p(2)+(M.w/2)+(f*M.fscale)*[0 1]);
set(M.title,'string',sprintf('t = %5.2f',t(end)));

function y = sigmoid(x)
y = 1/(1+exp(-x));






