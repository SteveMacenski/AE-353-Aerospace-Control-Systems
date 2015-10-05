function hw6prob02
clear all
clc;

% %%%%%%%%%%%%%%%%%%
% PARAMETERS
%
% - spin rate
params.n = 10;
% - transverse and axial moments of inertia
params.Jt = 10000;
params.Ja = 1000;
% - moment of inertia matrix
params.J = diag([10400,9600,1000]);
% - relative spin rate
params.lambda = ((params.Jt-params.Ja)/params.Jt)*params.n;
% - state-space system
params.A = [0 params.lambda; -params.lambda 0];
params.B = [1; 0];
params.C = [0 1];
% - time step
params.dt = 2e-2;
%
% %%%%%%%%%%%%%%%%%%

RunSimulation(5,params);

function [u,userdata] = ControlLoop(y,userdata,params)
%

A = [0 9;-9 0];
B = [1;0];
C = [0 1];
Qc = [1 0;0 2];
Rc = .999999;
Qo = 9999;
Ro = eye(2);

K = lqr(A,B,Qc,Rc);
L = lqr(A',C',inv(Ro),inv(Qo))';
persistent isFirstTime
if isempty(isFirstTime)
    isFirstTime = false;
    fprintf(1,'initialize control loop\n');
    userdata.K = K;
    userdata.kref = -inv(params.C*inv(params.A-params.B*userdata.K)*params.B);
    userdata.L = L;
    userdata.xhat = [0;0];
    
end
u = -userdata.K * userdata.xhat;
userdata.xhat = userdata.xhat + params.dt * (params.A * userdata.xhat + params.B * u - (userdata.L * (params.C * userdata.xhat - y)));



function RunSimulation(tmax,params)
% - define initial condition
w = [5; -5; params.n];
R = eye(3);
u = 0;
% - create array of times
t = 0:params.dt:tmax;
% - initialize userdata as an empty structure
userdata = struct;
% - get initial measurement
y = params.C*w(1:2,:);
% - initialize figure
fig = UpdateFigure([],w,R,u,t,1,params);
% - iterate to run simulation
for i=2:length(t)
    
    % Get input
    [u(i),userdata] = ControlLoop(y,userdata,params);
    
    % Get state
    [w(:,end+1),R] = OneStep(w(:,end),R,u(i),params);
    
    % Get measurement
    y = params.C*w(1:2,end);
    
    % Display
    fig = UpdateFigure(fig,w,R,u,t,i,params);
    
end

function [w,R] = OneStep(w,R,u,params)
[t,x] = ode45(@(t,x) f(t,x,u,params.J),[0 params.dt],[RtoX(R); w]);
w = x(end,10:end)';
x = x(end,1:9)';
R = XtoR(x);

function fig = UpdateFigure(fig,w,R,u,t,i,params)
if (isempty(fig))
    
    % - create box and axes
    Hb = 1;
    Mb = 6*(params.J(1,1)+params.J(2,2)-params.J(3,3))/(Hb^2);
    Wb = sqrt((6/Mb)*(params.J(1,1)-params.J(2,2)+params.J(3,3)));
    Lb = sqrt((6/Mb)*(-params.J(1,1)+params.J(2,2)+params.J(3,3)));
    [fig.p1,fig.faces,fig.colors] = makebox(Lb,Wb,Hb);
    fig.pAxis = 1.5*[0 0; 0 0; 0 1];
    fig.pAngVel = 1.5*[zeros(3,1) w/norm(w)];
    
    % - create figure
    clf;
    subplot(1,2,1);
    axis equal;
    axis(1.25*[-1 1 -1 1 -1 1]);
    axis manual;
    hold on;
    plot3(0,0,0,'k.','markersize',16);
    fig.hAxis = line(fig.pAxis(1,:),fig.pAxis(2,:),fig.pAxis(3,:));
    set(fig.hAxis,'linewidth',3,'color','r');
    fig.hAngVel = line(fig.pAngVel(1,:),fig.pAngVel(2,:),fig.pAngVel(3,:));
    set(fig.hAngVel,'linewidth',3,'color','k','linestyle','--');
    fig.hBox = patch('Vertices',fig.p1','Faces',fig.faces,...
              'CData',fig.colors,'FaceColor','flat');
    fig.hTitle = title(sprintf('t = %4.2f',0));
    lighting flat
    light('Position',[0 -2 -1])
    light('Position',[0 -2 1])
    light('Position',[5 3 1])
    light('Position',[-5 3 10])
    set(gca,'xtick',[],'ytick',[],'ztick',[]);
    subplot(2,2,2);
    axis([0 t(end) -max(abs(w)) max(abs(w))]);
    hold on;
    fig.hOmega1 = plot(0,w(1),'linewidth',3);
    fig.hOmega2 = plot(0,w(2),'linewidth',3);
    legend(' x_1',' x_2');
    xlabel('t');
    title('STATE');
    axis manual;
    subplot(2,2,4);
    axis([0 t(end) -50 50]);
    hold on;
    cmap = get(gca,'colororder');
    fig.hInput = plot(0,u(1),'linewidth',3,'color',cmap(3,:));
    legend(' u');
    xlabel('t');
    title('INPUT');
    axis manual;
    drawnow;
    
else
    
    p0 = R*fig.p1;
    set(fig.hBox,'Vertices',p0');
    pAxis0 = R*fig.pAxis;
    fig.pAngVel = 1.5*[zeros(3,1) w(:,i)/norm(w(:,i)')];
    pAngVel0 = R*fig.pAngVel;
    set(fig.hAxis,'xdata',pAxis0(1,:),'ydata',pAxis0(2,:),'zdata',pAxis0(3,:));
    set(fig.hAngVel,'xdata',pAngVel0(1,:),'ydata',pAngVel0(2,:),'zdata',pAngVel0(3,:));
    set(fig.hTitle,'string',sprintf('t = %4.2f',t(i)));
    set(fig.hInput,'xdata',t(1:i),'ydata',u(1:i));
    set(fig.hOmega1,'xdata',t(1:i),'ydata',w(1,1:i));
    set(fig.hOmega2,'xdata',t(1:i),'ydata',w(2,1:i));
    drawnow;
    
end

function xdot = f(t,x,u,J)
R = XtoR(x(1:9,1));
w = x(10:end,1);
Rdot = R*skew(w);
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