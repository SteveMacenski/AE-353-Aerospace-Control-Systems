function hw3prob02(K,x0)
K = [80 1/9];
x0 = [10;0];


clc;
Simulate(x0,K);

function Simulate(w0,K) %simulation by Professor Bretl, parameters and simulate by Steve Macenski

% DEFINE PARAMETERS
n = 10;
Jt = 10000;
Ja = 1000;
lambda = ((Jt-Ja)/Jt)*n;
R0 = eye(3);
w0 = [w0; n];
J = diag([Jt,Jt,Ja]);

% DEFINE THE TIME INTERVAL
tMax = 5;
tRate = 60;
t = linspace(0,tMax,tMax*tRate);

% SIMULATE THE SYSTEM
[t,x] = ode45(@(t,x) f(t,x,J,K),t,[RtoX(R0); w0]);
w = x(:,10:end);
x = x(:,1:9);
u = -K*(w(:,1:2)');

% CREATE A BOX AND SOME AXES
Hb = 1;
Mb = 6*(J(1,1)+J(2,2)-J(3,3))/(Hb^2);
Wb = sqrt((6/Mb)*(J(1,1)-J(2,2)+J(3,3)));
Lb = sqrt((6/Mb)*(-J(1,1)+J(2,2)+J(3,3)));
[p1,faces,colors] = makebox(Lb,Wb,Hb);
pAxis = 1.5*[0 0; 0 0; 0 1];
pAngVel = 1.5*[zeros(3,1) w0/norm(w0)];

% SETUP THE FIGURE
clf;
subplot(1,2,1);
axis equal;
axis(1.25*[-1 1 -1 1 -1 1]);
axis manual;
hold on;
plot3(0,0,0,'k.','markersize',16);
hAxis = line(pAxis(1,:),pAxis(2,:),pAxis(3,:));
set(hAxis,'linewidth',3,'color','r');
hAngVel = line(pAngVel(1,:),pAngVel(2,:),pAngVel(3,:));
set(hAngVel,'linewidth',3,'color','k','linestyle','--');
hBox = patch('Vertices',p1','Faces',faces,...
          'CData',colors,'FaceColor','flat');
hTitle = title(sprintf('t = %4.2f',0));
lighting flat
light('Position',[0 -2 -1])
light('Position',[0 -2 1])
light('Position',[5 3 1])
light('Position',[-5 3 10])
set(gca,'xtick',[],'ytick',[],'ztick',[]);
subplot(3,2,2);
axis([0 tMax -1.25*max(abs(w0)) 1.25*max(abs(w0))]);
hold on;
hOmega1 = plot(0,w0(1),'linewidth',3);
hOmega2 = plot(0,w0(2),'linewidth',3);
legend(' x_1',' x_2');
xlabel('t');
title('STATE');
axis manual;
subplot(3,2,4);
axis([0 tMax -30 30]);
hold on;
cmap = get(gca,'colororder');
hInput = plot(0,u(1),'linewidth',3,'color',cmap(3,:));
legend(' u');
xlabel('t');
title('INPUT');
axis manual;
subplot(3,2,6);
axis([0 tMax -0.5*max(abs(w0)) 0.5*max(abs(w0))]);
hold on;
hOutput = plot(0,w0(2),'linewidth',3,'color',cmap(4,:));
legend(' y');
xlabel('t');
title('OUTPUT');
axis manual;

% ANIMATE THE RESULTS
firsttime = 1;
i = 1;
dt = max(t)/(length(t)-1);
pause(1);
tic;
while (i<length(t))
    if (toc > dt)
        tic;
        i = i+1;
        R = XtoR(x(i,:));
        p0 = R*p1;
        set(hBox,'Vertices',p0');
        pAxis0 = R*pAxis;
        pAngVel = 1.5*[zeros(3,1) w(i,:)'/norm(w(i,:)')];
        pAngVel0 = R*pAngVel;
        set(hAxis,'xdata',pAxis0(1,:),'ydata',pAxis0(2,:),'zdata',pAxis0(3,:));
        set(hAngVel,'xdata',pAngVel0(1,:),'ydata',pAngVel0(2,:),'zdata',pAngVel0(3,:));
        set(hTitle,'string',sprintf('t = %4.2f',t(i)));
        set(hInput,'xdata',t(1:i),'ydata',u(1:i));
        set(hOmega1,'xdata',t(1:i),'ydata',w(1:i,1));
        set(hOmega2,'xdata',t(1:i),'ydata',w(1:i,2));
        set(hOutput,'xdata',t(1:i),'ydata',w(1:i,2));
        drawnow;
    end
end

function xdot = f(t,x,J,K)
R = XtoR(x(1:9,1));
w = x(10:end,1);
Rdot = R*skew(w);
u = -K*w(1:2,:);
wdot = J\([J(1,1)*u;0;0]-(skew(w)*J*w));
xdot = [RtoX(Rdot); wdot];

function S = skew(w)
S = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

function X = RtoX(R)
X = reshape(R,9,1);

function R = XtoR(X)
R = reshape(X,3,3);

function [verts,faces,colors] = makebox(x,y,z)
verts = [0 x x 0 0 x x 0; 0 0 0 0 y y y y; 0 0 z z 0 0 z z];
verts = verts - repmat([x/2; y/2; z/2],1,size(verts,2));
faces = [1 2 3 4; 2 6 7 3; 6 5 8 7; 5 1 4 8; 4 3 7 8; 5 6 2 1];
colors(:,:,1) = 1*ones(1,size(faces,1));
colors(:,:,2) = 1*ones(1,size(faces,1));
colors(:,:,3) = 0*ones(1,size(faces,1));
colors(1,5,1) = 1;
colors(1,5,2) = 0;
colors(1,5,3) = 0;
colors(1,3,1) = 0;
colors(1,3,2) = 0;
colors(1,3,3) = 1;
colors(1,2,1) = 0;
colors(1,2,2) = 1;
colors(1,2,3) = 0;



