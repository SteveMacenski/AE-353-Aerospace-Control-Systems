function hw4prob01

clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMPUTE STEP RESPONSE by Steve Macenski
R = 1e-6;
C1 = [1 0 0 0];
C2 = [0 1 0 0];
C3 = [-1 1 0 0];

Q1 = C1'*C1;
Q2 = C2'*C2;
Q3 = C3'*C3;

A = [0 0 1 0;0 0 0 1;-8 4 -1 .5;4 -4 .5 -.5];
B = [0; 0; 0; .5];

K1 = lqr(A,B,Q1,R);
K2 = lqr(A,B,Q2,R);
K3 = lqr(A,B,Q3,R);

Kref1 = (-(C1*inv((A-B*K1))*B))^-1;
Kref2 = -1/(C2*inv((A-B*K2))*B);
Kref3 = -1/(C3*inv((A-B*K3))*B);

t = linspace(0,10,1e3);

for i = 1:length(t)
     x(:,i) = inv(A-B*K3)*(expm((A-B*K3)*t(i))-eye(size(A,1)))*B*Kref3;;
end
y = C3*x;
u = -K3*x+Kref3;

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% by Professor Bretl
% VISUALIZE (don't change!)

% Define parameters
params.w1 = 1;
params.w2 = 1;
params.weq = 2.5;
params.kbH = 0.1;
params.kbS = 0.2;
params.fscale = 0.1;

% Animate
iCur = 1;
fig = UpdateFigure([],params,t,x,y,u,iCur);
pause;
tic
while (toc<t(end))
    iNext = find(toc>t,1,'last');
    if (~isempty(iNext)&&(iNext>iCur))
        iCur = iNext;
        fig = UpdateFigure(fig,params,t,x,y,u,iCur);
    end
end
fig = UpdateFigure(fig,params,t,x,y,u,length(t));

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function fig = UpdateFigure(fig,params,t,x,y,u,icur)
if (isempty(fig))
    
    clf;
    
    linewidth = 2;
    color = 'k';

    subplot(1,2,1);
    axis equal;
    hold on;
    axis([-1 1 0 10]);
    set(gcf,'renderer','opengl');
    set(gca,'fontsize',14);
    fig.mass(1)=line(-(1/2)+[0 0 1 1 0],x(1,icur)+params.weq+params.w1+[-params.w1 0 0 -params.w1 -params.w1],'linewidth',linewidth,'color',color);
    fig.mass(2)=line(-(1/2)+[0 0 1 1 0],x(2,icur)+2*params.weq+params.w1+[0 params.w2 params.w2 0 0],'linewidth',linewidth,'color',color);
    fig.spring(1)=patch(params.kbS+params.kbH*[-1 -1 1 1 -1],(params.weq+x(1,icur))*[0 1 1 0 0],'b','linewidth',linewidth/2,'facecolor',1*[0 0 1],'edgecolor',color,'facealpha',1-sigmoid(x(1,icur)));
    fig.damper(1)=patch(-params.kbS+params.kbH*[-1 -1 1 1 -1],(params.weq+x(1,icur))*[0 1 1 0 0],'r','linewidth',linewidth/2,'facecolor',1*[1 0 0],'edgecolor',color,'facealpha',1-sigmoid(x(3,icur)));
    fig.spring(2)=patch(params.kbS+params.kbH*[-1 -1 1 1 -1],params.weq+x(1,icur)+params.w1+(params.weq+x(2,icur)-x(1,icur))*[0 1 1 0 0],'b','linewidth',linewidth/2,'facecolor',1*[0 0 1],'edgecolor',color,'facealpha',1-sigmoid(x(2,icur)-x(1,icur)));
    fig.damper(2)=patch(-params.kbS+params.kbH*[-1 -1 1 1 -1],params.weq+x(1,icur)+params.w1+(params.weq+x(2,icur)-x(1,icur))*[0 1 1 0 0],'r','linewidth',linewidth/2,'facecolor',1*[1 0 0],'edgecolor',color,'facealpha',1-sigmoid(x(4,icur)-x(3,icur)));
    fig.force=plot([0 0],x(2,icur)+2*params.weq+params.w1+params.w2+(u(icur)*params.fscale)*[0 1],'m','linewidth',4);
    fig.title=title(sprintf('t = %5.2f',t(icur)));
    cmap = get(gca,'colororder');
    
    subplot(3,2,2);
    fig.output=plot(t(1),y(1),'color',cmap(1,:),'linewidth',2);
    axis([0 t(end) -0.5 1.5]);
    hold on;
    ylabel('y(t)');
    title('OUTPUT');
    set(gca,'fontsize',14);
    grid on;
    
    subplot(3,2,4);
    fig.input=plot(t(1),u(1),'color',cmap(2,:),'linewidth',2);
    axis([0 t(end) -20 20]);
    hold on;
    ylabel('u(t)');
    title('INPUT');
    set(gca,'fontsize',14);
    grid on;
    
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
    
else
    
    set(fig.mass(1),'ydata',x(1,icur)+params.weq+params.w1+[-params.w1 0 0 -params.w1 -params.w1]);
    set(fig.mass(2),'ydata',x(2,icur)+2*params.weq+params.w1+[0 params.w2 params.w2 0 0]);
    set(fig.spring(1),'ydata',(params.weq+x(1,icur))*[0 1 1 0 0],'facealpha',1-sigmoid(x(1,icur)));
    set(fig.damper(1),'ydata',(params.weq+x(1,icur))*[0 1 1 0 0],'facealpha',1-sigmoid(x(3,icur)));
    set(fig.spring(2),'ydata',params.weq+x(1,icur)+params.w1+(params.weq+x(2,icur)-x(1,icur))*[0 1 1 0 0],'facealpha',1-sigmoid(x(2,icur)-x(1,icur)));
    set(fig.damper(2),'ydata',params.weq+x(1,icur)+params.w1+(params.weq+x(2,icur)-x(1,icur))*[0 1 1 0 0],'facealpha',1-sigmoid(x(4,icur)-x(3,icur)));
    set(fig.force,'ydata',x(2,icur)+2*params.weq+params.w1+params.w2+(u(icur)*params.fscale)*[0 1]);
    
    
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







