function hw6prob03
clear all
clc;

% %%%%%%%%%%%%%%%%%%
% PARAMETERS
%
% - state-space system
params.A = [0 0 1 0; 0 0 0 1; -8 4 -1 0.5; 4 -4 0.5 -0.5];
params.B = [0; 0; 0; 0.5];
params.C = [0 1 0 0];
% - time step
params.dt = 1e-3;
%
% %%%%%%%%%%%%%%%%%%

RunSimulation(6,params);


function [u,userdata] = ControlLoop(y,userdata,params)
%
% 
%
A = [0 0 1 0;0 0 0 1;-8 4 -1 .5; 4 -4 .5 -.5];
B = [0;0;0;.5];
C = [0 1 0 0];
Qc = [100 0 0 0;0 1000 0 0;0 0 .0001 0; 0 0 0 .01];
Rc = 1;
Qo = 10000;
Ro = [.01 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 100000];


K = lqr(A,B,Qc,Rc);
L = lqr(A',C',inv(Ro),inv(Qo))';

persistent isFirstTime
if isempty(isFirstTime)
    isFirstTime = false;
    fprintf(1,'initialize control loop\n');
    userdata.K = K;
    userdata.kref = -inv(params.C*inv(params.A-params.B*userdata.K)*params.B);
    userdata.L = L;
    userdata.xhat = [1.1;2.9;-.1;.1];

end
u = -userdata.K * userdata.xhat;
userdata.xhat = userdata.xhat + params.dt * (params.A * userdata.xhat + params.B * u - (userdata.L * (params.C * userdata.xhat - y)));





function RunSimulation(tmax,params)
% - initial condition
x = [1;3;0;0;0;0];
u = 0;
% - create array of times
t = 0:params.dt:tmax;
% - initialize userdata as an empty structure
userdata = struct;
% - get initial measurement
y = x(2,end);
% - discretize the system
m2 = 2;
k3 = 1e2;
b3 = 1e0;
m3 = 2;
A = [0 0 1 0 0 0; 0 0 0 1 0 0; -8 4 -1 0.5 0 0; 4 -4-(k3/m2) 0.5 -0.5-(b3/m2) k3/m2 b3/m2; 0 0 0 0 0 1; 0 k3/m3 0 b3/m3 -k3/m3 -b3/m3];
B = [0; 0; 0; 0.5; 0; 0];
C = [0 0 0 0 1 0];
Ad = expm(A*params.dt);
Bd = inv(A)*(expm(A*params.dt)-eye(size(A)))*B;
% - number of frames to skip when visualizing
nskip = 25;
% - initialize figure
fig = UpdateDisplay([],x,y,u,t,1);
% - iterate to run simulation
for i=2:length(t)
    
    % Get input
    [u(:,end+1),userdata] = ControlLoop(y(:,end),userdata,params);
    
    % Get state
    x(:,end+1) = Ad*x(:,end)+Bd*u(:,end);
    
    % Get measurement
    y(:,end+1) = x(2,end);
    
    % Display
    if (~mod(i,nskip))
        fig = UpdateDisplay(fig,x,y,u,t,i);
    end
    
end

function fig = UpdateDisplay(fig,x,y,u,t,icur)
if (isempty(fig))
    
    fig.w1 = 1;
    fig.w2 = 1;
    fig.weq = 2.5;
    fig.kbH = 0.1;
    fig.kbS = 0.2;
    fig.fscale = 0.1;
    
    clf;
    
    linewidth = 2;
    color = 'k';

    subplot(1,2,1);
    axis equal;
    hold on;
    axis([-1 1 0 10]);
    set(gcf,'renderer','opengl');
    set(gca,'fontsize',14);
    fig.mass(1)=line(-(1/2)+[0 0 1 1 0],x(1,icur)+fig.weq+fig.w1+[-fig.w1 0 0 -fig.w1 -fig.w1],'linewidth',linewidth,'color',color);
    fig.mass(2)=line(-(1/2)+[0 0 1 1 0],x(2,icur)+2*fig.weq+fig.w1+[0 fig.w2 fig.w2 0 0],'linewidth',linewidth,'color',color);
    fig.spring(1)=patch(fig.kbS+fig.kbH*[-1 -1 1 1 -1],(fig.weq+x(1,icur))*[0 1 1 0 0],'b','linewidth',linewidth/2,'facecolor',1*[0 0 1],'edgecolor',color,'facealpha',1-sigmoid(x(1,icur)));
    fig.damper(1)=patch(-fig.kbS+fig.kbH*[-1 -1 1 1 -1],(fig.weq+x(1,icur))*[0 1 1 0 0],'r','linewidth',linewidth/2,'facecolor',1*[1 0 0],'edgecolor',color,'facealpha',1-sigmoid(x(3,icur)));
    fig.spring(2)=patch(fig.kbS+fig.kbH*[-1 -1 1 1 -1],fig.weq+x(1,icur)+fig.w1+(fig.weq+x(2,icur)-x(1,icur))*[0 1 1 0 0],'b','linewidth',linewidth/2,'facecolor',1*[0 0 1],'edgecolor',color,'facealpha',1-sigmoid(x(2,icur)-x(1,icur)));
    fig.damper(2)=patch(-fig.kbS+fig.kbH*[-1 -1 1 1 -1],fig.weq+x(1,icur)+fig.w1+(fig.weq+x(2,icur)-x(1,icur))*[0 1 1 0 0],'r','linewidth',linewidth/2,'facecolor',1*[1 0 0],'edgecolor',color,'facealpha',1-sigmoid(x(4,icur)-x(3,icur)));
    fig.force=plot([0 0],x(2,icur)+2*fig.weq+fig.w1+fig.w2+(u(icur)*fig.fscale)*[0 1],'m','linewidth',4);
    fig.title=title(sprintf('t = %5.2f',t(icur)));
    cmap = get(gca,'colororder');
    box on;
    
    subplot(3,2,2);
    fig.output=plot(t(1),y(1),'color',cmap(1,:),'linewidth',2);
    axis([0 t(end) -0.5 1.5]);
    hold on;
    ylabel('y(t)');
    title('OUTPUT');
    set(gca,'fontsize',14);
    grid on;
    box on;
    
    subplot(3,2,4);
    fig.input=plot(t(1),u(1),'color',cmap(2,:),'linewidth',2);
    axis([0 t(end) -20 20]);
    hold on;
    ylabel('u(t)');
    title('INPUT');
    set(gca,'fontsize',14);
    grid on;
    box on;
    
    subplot(3,2,6);
    axis([0 t(end) -3 3]);
    hold on;
    for i=1:4
        fig.state(i)=plot(t(1),x(i,1),'color',cmap(2+i,:),'linewidth',2);
    end
    xlabel('t');
    ylabel('x(t)');
    title('STATE');
    legend('x1','x2','x3','x4');
    set(gca,'fontsize',14);
    grid on;
    box on;
    
else
    
    set(fig.mass(1),'ydata',x(1,icur)+fig.weq+fig.w1+[-fig.w1 0 0 -fig.w1 -fig.w1]);
    set(fig.mass(2),'ydata',x(2,icur)+2*fig.weq+fig.w1+[0 fig.w2 fig.w2 0 0]);
    set(fig.spring(1),'ydata',(fig.weq+x(1,icur))*[0 1 1 0 0],'facealpha',1-sigmoid(x(1,icur)));
    set(fig.damper(1),'ydata',(fig.weq+x(1,icur))*[0 1 1 0 0],'facealpha',1-sigmoid(x(3,icur)));
    set(fig.spring(2),'ydata',fig.weq+x(1,icur)+fig.w1+(fig.weq+x(2,icur)-x(1,icur))*[0 1 1 0 0],'facealpha',1-sigmoid(x(2,icur)-x(1,icur)));
    set(fig.damper(2),'ydata',fig.weq+x(1,icur)+fig.w1+(fig.weq+x(2,icur)-x(1,icur))*[0 1 1 0 0],'facealpha',1-sigmoid(x(4,icur)-x(3,icur)));
    set(fig.force,'ydata',x(2,icur)+2*fig.weq+fig.w1+fig.w2+(u(icur)*fig.fscale)*[0 1]);
    
    
    set(fig.title,'string',sprintf('t = %5.2f',t(icur)));
    
    set(fig.output,'xdata',t(1:icur),'ydata',y(1:icur));
    set(fig.input,'xdata',t(1:icur),'ydata',u(1:icur));
    for i=1:4
        set(fig.state(i),'xdata',t(1:icur),'ydata',x(i,1:icur));
    end
    
    
end
drawnow;

function y = sigmoid(x)
y = 1/(1+exp(-x));

