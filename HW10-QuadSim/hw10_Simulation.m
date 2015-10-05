function hw10_Simulation

% Clean up workspace
clc
clear all
close all

% Run simulation
%
%  argument #1 : tmax = time for contest
%  argument #2 : nskip = number of frames to skip for display
%  argument #3 : dodisplay = true or false, whether or not to animate
%
[simdata,userdata] = RunSimulation(120,2,true);

% Useful things in simdata:
%
%  simdata.t = time
%  simdata.robot(k) = structure that describes the k'th robot
%  simdata.params = sturcture that describes simulation parameters

% Useful things in userdata:
%
%  userdata{k} = structure with whatever the k'th control system put there

% Example of a useful thing to plot:
%
%  figure;
%  k = 1;
%  plot(simdata.t,simdata.robot(k).fR,'k-',simdata.t,simdata.robot(k).fL,'r-');
%  legend('fR','fL');
%  title(sprintf('right and left rotor forces for robot %d',k));

function [simdata,userdata,params] = RunSimulation(tmax,nskip,dodisplay)

% %%%%%%%%%%%%%%%%%%
% PARAMETERS
%
% - gravity
simdata.params.g = 9.81;
% - length of each spar
simdata.params.w = 0.25;
% - mass of robot and package
simdata.params.m = 1;
simdata.params.mpackage = 0.15;
% - moment of inertia of robot and package
simdata.params.J = 0.1;
simdata.params.Jpackage = 0.005;
% - minimum allowable rotor force
simdata.params.fmin = 0;
% - maximum allowable rotor force
simdata.params.fmax = 10;
% - time step
simdata.params.dt = 1e-2;
% - workspace bounds in the form [xmin xmax ymin ymax]
simdata.params.workspace = [-3 3 0 6.48];
% - radius of each target
simdata.params.rtarget = .15;
% - required dwell time at each target
simdata.params.ttarget = 1;
% - time remaining
simdata.params.timeremaining = tmax;
% - nominal initial position of each robot
simdata.params.pstart = [0; 2];
% - radius of robot "basket" that must be kept inside target
simdata.params.rbasket = 0.1;
% - rate at which battery is depleted
simdata.params.kbattery = 0.03;
% - rate at which battery is charged
simdata.params.chargerate = 30;
%
%
% %%%%%%%%%%%%%%%%%%

% %%%%%%%%%%%%%%%%%%
% INTERNALS
%
% - region within which "to" targets will be placed
simdata.internal.insetworkspace = [-2.5 2.5 1.5 4.98];
% - region within which "from" targets will be placed
simdata.internal.homex = [-2.5 2.5];
simdata.internal.homey = [1];
% - geometry of robot
simdata.internal.sparheight = 0.05*simdata.params.w;
simdata.internal.rotorheight = 3*simdata.internal.sparheight;
simdata.internal.rotorwidth = 0.5*simdata.params.w;
simdata.internal.nameoffset = 2*simdata.internal.rotorheight;
% - students
addpath('students');
simdata.internal.students = GetStudents;
simdata.internal.nstudents = length(simdata.internal.students);
% - contest time
simdata.internal.tmax = tmax;
clear tmax
simdata.internal.imax = 1+(simdata.internal.tmax/simdata.params.dt);
% - figure is this fraction of screen width
simdata.internal.figuresize = 0.9;
% - amount of measurement noise
simdata.internal.noisemag = [8e-2;8e-2;1e-3];
% - radius of robot payload
simdata.internal.rpayload = 0.07;
%
% %%%%%%%%%%%%%%%%%%

% %%%%%%%%%%%%%%%%%%
% SIMULATE
%
% - initial position and orientation offset for all robots
dq = [0.25;0.25;0.25].*(-1+2*rand(3,1));
% - each robot
for k=1:simdata.internal.nstudents
    % - initial inputs
    simdata.robot(k).fR(1) = 0;
    simdata.robot(k).fL(1) = 0;
    % - control system function handle
    simdata.robot(k).GetInput = str2func(simdata.internal.students(k).filename);
    % - control system name
    simdata.robot(k).name = simdata.internal.students(k).lastname;
    % - time for which the robot has dwelled at the current target
    simdata.robot(k).todwelltime = 0;
    simdata.robot(k).fromdwelltime = 0;
    % - empty structure for user to modify
    userdata{k} = struct;
    userdata{k}.isFirstTime = true;
    % - the current "to" target (if empty, then robot has no package)
    simdata.robot(k).pTo = [];
    % - the array of "from" and "battery" targets
    xL = simdata.internal.homex(1);
    xR = simdata.internal.homex(2);
    y = simdata.internal.homey;
    n = simdata.internal.nstudents;
    simdata.robot(k).pFrom = [(1-((k)/(n+1)))*xL+((k)/(n+1))*xR; simdata.params.workspace(4)-y];
    simdata.robot(k).pBattery = [(1-((k)/(n+1)))*xL+((k)/(n+1))*xR; y];
    clear xL xR y n
    % - initial state
    simdata.robot(k).q(:,1) = [simdata.params.pstart; 0; 0; 0; 0];
    simdata.robot(k).q(1:3,1) = simdata.robot(k).q(1:3,1)+dq;
    % - battery charge
    simdata.robot(k).battery = 100;
    % - is the battery charging
    simdata.robot(k).charging = false;
    % - has the robot crashed
    simdata.robot(k).crashed = false;
    % - how many packages remain to be delivered
    simdata.robot(k).delivered = 10;
    % - time at which all packages have been delivered
    simdata.robot(k).donetime = inf;
end
clear dq
% - time
simdata.t(1) = 0;
% - figure
if (dodisplay)
    simdata.fig = UpdateFigure(1,simdata,[]);
end
% - loop
for i=2:simdata.internal.imax
    % - time
    simdata.t(i) = (i-1)*simdata.params.dt;
    simdata.params.timeremaining = simdata.internal.tmax-simdata.t(i);
    % - dynamics
    [simdata,userdata,done] = OneStep(i,simdata,userdata);
    % - update figure
    if (dodisplay&&(~mod(i-1,nskip)))
        simdata.fig = UpdateFigure(i,simdata,simdata.fig);
    end
    % - check if done
    if (done)
        nskip = 1000;
    end
end
if (dodisplay)
    simdata.fig = UpdateFigure(simdata.internal.imax,simdata,simdata.fig);
end
%
% %%%%%%%%%%%%%%%%%%

function c = GetColor(k)
cmap =[0    0.4470    0.7410
    0.8500    0.3250    0.0980
    0.9290    0.6940    0.1250
    0.4940    0.1840    0.5560
    0.4660    0.6740    0.1880
    0.3010    0.7450    0.9330
    0.6350    0.0780    0.1840];
k = mod(k-1,7)+1;
c = cmap(k,:);

function [x,y] = GetRandomPoint(xlim,ylim)
x = xlim(1)+rand*(xlim(2)-xlim(1));
y = ylim(1)+rand*(ylim(2)-ylim(1));

function students = GetStudents
addpath('students');
listing = dir('students');
fprintf(1,'\nFINDING CONTROL SYSTEMS\n');
fprintf(1,' filename : lastname\n');
nstudents = 0;
for i=1:length(listing)
    if (~listing(i).isdir)
        [tmp1,student.filename,tmp2] = fileparts(listing(i).name);
        if (~isempty(student.filename))
            nstudents = nstudents + 1;
            data = textscan(student.filename,'hw10_%s');
            student.lastname = data{1}{1};
            students(nstudents) = student;
            fprintf(1,'  %s : %s\n',student.filename,student.lastname);
        end
    end
end
fprintf(1,'\n\n');

function res = InRectangle(q,qrect)
res = 1;
if ((q(1)<qrect(1))||(q(1)>qrect(2))||(q(2)<qrect(3))||(q(2)>qrect(4)))
    res = 0;
end

function [simdata,userdata,done] = OneStep(i,simdata,userdata)
done = 1;
% - robots
for k=1:simdata.internal.nstudents
    
    % - check if done : finished
    if ~isinf(simdata.robot(k).donetime)
        simdata.robot(k).q(:,i) = simdata.robot(k).q(:,i-1);
        continue;
    end
    
    % - check if done : crashed
    if simdata.robot(k).crashed
        simdata.robot(k).q(:,i) = simdata.robot(k).q(:,i-1);
        continue;
    else
        q = simdata.robot(k).q(1:2,i-1);
        if ~InRectangle(q,simdata.params.workspace)
            simdata.robot(k).q(:,i) = simdata.robot(k).q(:,i-1);
            simdata.robot(k).crashed = true;
            continue;
        end
    end
    
    % - not done
    done=0;
    
    % - battery dwell
    dBattery = norm(simdata.robot(k).q(1:2,i-1)-simdata.robot(k).pBattery);
    if (dBattery<(simdata.params.rtarget-simdata.params.rbasket))
        simdata.robot(k).charging = true;
        simdata.robot(k).battery = simdata.robot(k).battery + simdata.params.chargerate*simdata.params.dt;
    else
        simdata.robot(k).charging = false;
    end
    
    % - from dwell
    dHome = norm(simdata.robot(k).q(1:2,i-1)-simdata.robot(k).pFrom);
    if (dHome<(simdata.params.rtarget-simdata.params.rbasket))
        simdata.robot(k).fromdwelltime = simdata.robot(k).fromdwelltime + simdata.params.dt;
        if (isempty(simdata.robot(k).pTo) && (simdata.robot(k).fromdwelltime > simdata.params.ttarget))
            
            [x,y] = GetRandomPoint(simdata.internal.insetworkspace(1:2),simdata.internal.insetworkspace(3:4));
            simdata.robot(k).pTo = [x;y];
            
        end
    else
        simdata.robot(k).fromdwelltime = 0;
    end
    
    % - to dwell
    if (~isempty(simdata.robot(k).pTo))
        dTarget = norm(simdata.robot(k).q(1:2,i-1)-simdata.robot(k).pTo);
        if (dTarget<(simdata.params.rtarget-simdata.params.rbasket))
            simdata.robot(k).todwelltime = simdata.robot(k).todwelltime + simdata.params.dt;
            if (simdata.robot(k).todwelltime > simdata.params.ttarget)
                simdata.robot(k).pTo = [];
                simdata.robot(k).todwelltime = 0;
                simdata.robot(k).delivered = simdata.robot(k).delivered-1;
                if (simdata.robot(k).delivered == 0)
                    simdata.robot(k).donetime = simdata.t(i);
                end
            end
        else
            simdata.robot(k).todwelltime = 0;
        end
    end
    
    % - simulate measurements
    y = simdata.robot(k).q([1 2 6],i-1)+simdata.internal.noisemag.*randn(3,1);
    
    % - get inputs from user control system
    [fR,fL,userdata{k}] = simdata.robot(k).GetInput(y(1),y(2),y(3),...
                                                    simdata.robot(k).battery,...
                                                    simdata.robot(k).pTo,...
                                                    simdata.robot(k).pFrom,...
                                                    simdata.robot(k).pBattery,...
                                                    userdata{k},...
                                                    simdata.params);
    
    % - bound the right and left rotor forces
    fR = BoundForce(fR,simdata.params.fmin,simdata.params.fmax);
    fL = BoundForce(fL,simdata.params.fmin,simdata.params.fmax);
    
    % - deplete battery
    simdata.robot(k).battery = simdata.robot(k).battery - simdata.params.kbattery*simdata.params.dt*(fR.^2+fL.^2);
    if (simdata.robot(k).battery<0)
        simdata.robot(k).battery = 0;
        fR = 0;
        fL = 0;
    elseif (simdata.robot(k).battery>100)
        simdata.robot(k).battery=100;
    end
    
    % - integrate
    if (isempty(simdata.robot(k).pTo))
        % not carrying a package (light)
        [t,q] = ode45(@(t,q) fqLight(t,q,fR,fL,simdata.params),[0 simdata.params.dt],simdata.robot(k).q(:,i-1));
    else
        % carrying a package (heavy)
        [t,q] = ode45(@(t,q) fqHeavy(t,q,fR,fL,simdata.params),[0 simdata.params.dt],simdata.robot(k).q(:,i-1));
    end
    q = q(end,:)';
    
    % - put all the results into simdata
    simdata.robot(k).y(:,i-1) = y;
    simdata.robot(k).q(:,i) = q;
    simdata.robot(k).fR(:,i) = fR;
    simdata.robot(k).fL(:,i) = fL;
    
end

function f = BoundForce(f,fmin,fmax)
if (f>fmax)
    f = fmax;
elseif (f<fmin)
    f = fmin;
end

function dqdt = fqLight(t,q,fR,fL,params)
dqdt = [q(4:6);
        -(fR+fL)*sin(q(3))/params.m;
        ((fR+fL)*cos(q(3))-(params.m*params.g))/params.m;
        params.w*(fR-fL)/params.J];
function dqdt = fqHeavy(t,q,fR,fL,params)
dqdt = [q(4:6);
        -(fR+fL)*sin(q(3))/(params.m+params.mpackage);
        ((fR+fL)*cos(q(3))-((params.m+params.mpackage)*params.g))/(params.m+params.mpackage);
        params.w*(fR-fL)/(params.J+params.Jpackage)];

function R = RotMat(h)
R = [cos(h) -sin(h); sin(h) cos(h)];

function p = RigidBodyTransform(p,R,d)
p = (R*p)+repmat(d,1,size(p,2));

function fig = UpdateFigure(i,simdata,fig)
if isempty(fig)
    
    % - create figure
    scrsz = get(0,'screensize');
    w = floor((simdata.internal.figuresize*scrsz(3))/5)*5;
    h = 0.6*w;
	fig.fig = figure(...
        'position',[10 10 w h],...
        'dockcontrols','off','menubar','none','toolbar','none',...
        'resize','off','renderer','painters');
    
    % - create axes
    fig.axes = axes(...
        'fontname','fixedwidth',...
        'fontsize',18,'position',[0.03 0.05 0.5 0.9]);
    axis equal;
    hold on;
    w = 3;
    h = ((0.9*0.6)/0.5)*w;
    set(fig.axes,'xlim',[-w w],'ylim',[0 2*h]);
    set(fig.axes,'xtick',[],'ytick',[],'box','on');
    clear w h scrsz
    
    % - targets
    for k=1:simdata.internal.nstudents
        
        fig.robot(k).color = GetColor(k);
        fig.target(k).bound = rectangle('position',[0,0,1,1],...
                                        'visible','off',...
                                        'curvature',[0 0],...
                                        'edgecolor','k',...
                                        'facecolor','w',...
                                        'linewidth',1);
        fig.target(k).point = rectangle('position',[0,0,1,1],...
                                        'visible','off',...
                                        'curvature',[1 1],...
                                        'edgecolor',fig.robot(k).color,...
                                        'facecolor','w',...
                                        'linewidth',1);
        
    end
    
    % - template robot
    fig.template.spar = GetRectangle(simdata.params.w,simdata.internal.sparheight);
    rotor = GetRectangle(simdata.internal.rotorwidth,simdata.internal.rotorheight);
    fig.template.rightrotor = [rotor(1,:)+simdata.params.w; rotor(2,:)];
    fig.template.leftrotor = [rotor(1,:)-simdata.params.w; rotor(2,:)];
    
    % - homes
    for k=1:simdata.internal.nstudents
        
        fig.robot(k).from = rectangle('position',[simdata.robot(k).pFrom(1)-simdata.params.rtarget simdata.robot(k).pFrom(2)-simdata.params.rtarget 2*simdata.params.rtarget 2*simdata.params.rtarget],...
                                      'curvature',[1 1],...
                                      'facecolor','w',...
                                      'edgecolor',fig.robot(k).color,...
                                      'linewidth',1,...
                                      'visible','on');
        fig.robot(k).battery = rectangle('position',[simdata.robot(k).pBattery(1)-simdata.params.rtarget simdata.robot(k).pBattery(2)-simdata.params.rtarget 2*simdata.params.rtarget 2*simdata.params.rtarget],...
                                      'curvature',[1 1],...
                                      'facecolor','w',...
                                      'edgecolor',fig.robot(k).color,...
                                      'linewidth',1,...
                                      'visible','on');
        
    end
    
    % - specific robots -- geometry
    for k=1:simdata.internal.nstudents
        
        
        
        R = RotMat(simdata.robot(k).q(3,i));
        d = simdata.robot(k).q(1:2,i);
        
        p = RigidBodyTransform(fig.template.spar,R,d);
        fig.robot(k).spar = patch(p(1,:),p(2,:),fig.robot(k).color);
        p = RigidBodyTransform(fig.template.rightrotor,R,d);
        fig.robot(k).rightrotor = patch(p(1,:),p(2,:),fig.robot(k).color);
        p = RigidBodyTransform(fig.template.leftrotor,R,d);
        fig.robot(k).leftrotor = patch(p(1,:),p(2,:),fig.robot(k).color);
        
        fig.robot(k).basket = rectangle('position',[d(1)-simdata.params.rbasket d(2)-simdata.params.rbasket 2*simdata.params.rbasket 2*simdata.params.rbasket],...
                                        'curvature',[1 1],...
                                        'facecolor','w',...
                                        'edgecolor',fig.robot(k).color,...
                                        'linewidth',2);
        
        fig.robot(k).payload = rectangle('position',[d(1)-simdata.internal.rpayload d(2)-simdata.internal.rpayload 2*simdata.internal.rpayload 2*simdata.internal.rpayload],...
                                         'curvature',[1 1],...
                                         'facecolor',[0.8 0.7 0.6],...
                                         'edgecolor','k',...
                                         'linewidth',1,...
                                         'visible','off');
        
        
    end
    
    % - specific robots -- name
    for k=1:simdata.internal.nstudents
        
        p = simdata.robot(k).q(1:2,i)+RotMat(simdata.robot(k).q(3,i))*[0;-simdata.internal.nameoffset];
        fig.robot(k).name = text(p(1),p(2),simdata.robot(k).name,...
            'interpreter','none','fontsize',12,...
            'verticalalignment','top','horizontalalignment','center',...
            'rotation',radtodeg(simdata.robot(k).q(3,i)));
        
    end
    
    
    % - title (time reamining)
    fig.title = title(sprintf('TIME REMAINING : %8.2f',simdata.params.timeremaining));
    
    % - scoreboard axes
    fig.scoreboard = axes(...
        'fontname','fixedwidth',...
        'fontsize',18,'position',[0.97-0.41 0.05 0.41 0.9]);
    set(fig.scoreboard,'xtick',[],'ytick',[],'box','on');
    
    % - scores
    fSize = 12;
    text(0.025,0.96,'RACING','fontsize',1.5*fSize); 
    text(0.525,0.96,'FINISHED','fontsize',1.5*fSize);
    for k=1:simdata.internal.nstudents
        score(k,:) = simdata.robot(k).delivered;
    end
    [tmp,K] = sortrows(score);
    nRows = 30;
    fSize = 12;
    w = [0.05*ones(nRows,1); 0.55*ones(nRows,1)];
    h = repmat(linspace(0.95,0.05,nRows)',2,1);
    for k=1:simdata.internal.nstudents
        fig.score(k) = text(0.05,0.95-k*0.03,...
            sprintf('%d : %5.2f : %5.1f%% : %s\n',...
             score(K(k),1),...
             max(0,simdata.params.ttarget-max(simdata.robot(K(k)).todwelltime,simdata.robot(K(k)).fromdwelltime)),...
             simdata.robot(K(k)).battery,...
             simdata.robot(K(k)).name),...
            'fontsize',fSize,'interpreter','none','color','k');
    end
    
else
    
    % - robots
    for k=1:simdata.internal.nstudents
        
        if (~isinf(simdata.robot(k).donetime))
            set(fig.robot(k).spar,'visible','off');
            set(fig.robot(k).rightrotor,'visible','off');
            set(fig.robot(k).leftrotor,'visible','off');
            set(fig.robot(k).name,'visible','off');
            set(fig.robot(k).basket,'visible','off');
            set(fig.robot(k).from,'visible','off');
            set(fig.robot(k).battery,'visible','off');
            set(fig.target(k).point,'visible','off');
            set(fig.target(k).bound,'visible','off');
            set(fig.robot(k).payload,'visible','off');
            continue;
        end
        
        if (simdata.robot(k).crashed)
            set(fig.robot(k).basket,'visible','off');
            set(fig.robot(k).from,'visible','off');
            set(fig.robot(k).battery,'visible','off');
            set(fig.target(k).point,'visible','off');
            set(fig.target(k).bound,'visible','off');
            set(fig.robot(k).payload,'visible','off');
        end
        
        R = RotMat(simdata.robot(k).q(3,i));
        d = simdata.robot(k).q(1:2,i);
        
        p = RigidBodyTransform(fig.template.spar,R,d);
        set(fig.robot(k).spar,'xdata',p(1,:),'ydata',p(2,:));
        p = RigidBodyTransform(fig.template.rightrotor,R,d);
        set(fig.robot(k).rightrotor,'xdata',p(1,:),'ydata',p(2,:));
        p = RigidBodyTransform(fig.template.leftrotor,R,d);
        set(fig.robot(k).leftrotor,'xdata',p(1,:),'ydata',p(2,:));
        
        p = RigidBodyTransform([0;-simdata.internal.nameoffset],R,d);
        set(fig.robot(k).name,'position',p,'rotation',radtodeg(simdata.robot(k).q(3,i)));
        
        set(fig.robot(k).basket,'position',[d(1)-simdata.params.rbasket d(2)-simdata.params.rbasket 2*simdata.params.rbasket 2*simdata.params.rbasket]);
        
        if (simdata.robot(k).fromdwelltime>0)
            set(fig.robot(k).from,'facecolor',1-(0.5*(1-fig.robot(k).color)));
        else
            set(fig.robot(k).from,'facecolor','w');
        end
        
        if (simdata.robot(k).charging)
            set(fig.robot(k).battery,'facecolor',1-(0.5*(1-fig.robot(k).color)));
        else
            set(fig.robot(k).battery,'facecolor','w');
        end
        
        if (simdata.robot(k).todwelltime>0)
            set(fig.target(k).point,'facecolor',1-(0.5*(1-fig.robot(k).color)));
        else
            set(fig.target(k).point,'facecolor','w');
        end
        
        if (isempty(simdata.robot(k).pTo))
            set(fig.target(k).point,'visible','off');
            set(fig.target(k).bound,'visible','off');
            set(fig.robot(k).payload,'visible','off');
        else
            set(fig.target(k).point,'position',[simdata.robot(k).pTo(1)-simdata.params.rtarget ,simdata.robot(k).pTo(2)-simdata.params.rtarget ,2*simdata.params.rtarget ,2*simdata.params.rtarget],...
                                    'visible','on');
            set(fig.target(k).bound,'position',[simdata.robot(k).pTo(1)-1.5*simdata.params.rtarget,simdata.robot(k).pTo(2)-1.5*simdata.params.rtarget ,2*1.5*simdata.params.rtarget ,2*1.5*simdata.params.rtarget],...
                                    'visible','on');
            set(fig.robot(k).payload,'position',[d(1)-simdata.internal.rpayload d(2)-simdata.internal.rpayload 2*simdata.internal.rpayload 2*simdata.internal.rpayload],...
                                     'visible','on');
        end

    end
    
    % - title
    set(fig.title,'string',sprintf('TIME REMAINING : %8.2f',simdata.params.timeremaining));
    
    % -- scores : racing
    K = [];
    S = [];
    for k=1:simdata.internal.nstudents
        if isinf(simdata.robot(k).donetime)
            K = [K k];
            S = [S simdata.robot(k).delivered];
        end
    end
    if (~isempty(K))
        [tmp,J] = sort(S,'ascend');
        for j=1:length(J)
            set(fig.score(K(J(j))),...
                'position',[0.05,0.95-j*0.03,0],...
                'string',sprintf('%d : %5.2f : %5.1f%% : %s\n',...
                    simdata.robot(K(J(j))).delivered,...
                    max(0,simdata.params.ttarget-max(simdata.robot(K(J(j))).todwelltime,simdata.robot(K(J(j))).fromdwelltime)),...
                    simdata.robot(K(J(j))).battery,...
                    simdata.robot(K(J(j))).name));
            if (simdata.robot(K(J(j))).crashed)
                set(fig.score(K(J(j))),'color','r');
            end
        end
    end
    % -- scores : finished
    K = [];
    S = [];
    for k=1:simdata.internal.nstudents
        if ~isinf(simdata.robot(k).donetime)
            K = [K k];
            S = [S simdata.robot(k).donetime];
        end
    end
    if (~isempty(K))
        [tmp,J] = sort(S,'ascend');
        for j=1:length(J)
            set(fig.score(K(J(j))),...
                'position',[0.55,0.95-j*0.03,0],...
                'string',sprintf('%7.2f : %s\n',...
                    simdata.robot(K(J(j))).donetime,...
                    simdata.robot(K(J(j))).name));
        end
    end
    
end
drawnow;

function p = GetRectangle(w,h)
p = [-w w w -w -w; -h -h h h -h];
