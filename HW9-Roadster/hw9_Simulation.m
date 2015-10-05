function hw9_Simulation

% Clean up workspace
clc
clear all
close all

% Run simulation
%
% userdata for car #k is in car(k).userdata)
%
[car,params] = RunSimulation('testroad.mat',true);
save('results.mat');
savefig('results.fig');

function [car,params] = RunSimulation(filename,dodisplay)

% %%%%%%%%%%%%%%%%%%
% SET PARAMETERS Written by Steven Macenski
%
% - half the distance between the two wheels
params.b = 0.25;
% - radius of each wheel
params.r = 0.15;
% - torque constant
params.ktau = 1;
% - magnitude of maximum allowable torque in each wheel
params.taumax = 10;
% - time step
params.dt = 1e-2;
% - students
addpath('students');
params.students = GetStudents;
params.nstudents = length(params.students);
% - width of road
params.roadwidth = 2;
% - maximum time allowed to finish course
params.tmax = 120;
%
% %%%%%%%%%%%%%%%%%%

% %%%%%%%%%%%%%%%%%%
% LOAD THE ROAD
%
load(filename);
clear filename
%
% %%%%%%%%%%%%%%%%%%

% %%%%%%%%%%%%%%%%%%
% SIMULATE
%
for k=1:params.nstudents %written by Professor Bretl
    % - initial state
    car(k).q = [0; 0; 0; 0; 0];
    % - initial inputs
    car(k).tauR(1) = 0;
    car(k).tauL(1) = 0;
    
    % - control system function handle
    car(k).GetInput = str2func(params.students(k).filename);
    % - control system name
    car(k).name = params.students(k).name;
    % - event time
    car(k).eventtime = nan;
    % - result flag
    car(k).result = nan;
    % - arc-length
    car(k).sdes = 0;
    % - vdes, wdes, qdes
    car(k).vdes = 1;
    [car(k).qdes,car(k).wdes] = WhereAmI(car(k).sdes,car(k).vdes,road);
    % - initial error, expressed in body-fixed reference frame
    car(k).e = GetError(car(k).q,car(k).qdes);
    % - color
    car(k).color = GetColor(k);
    % - closest point on road
    car(k).sclosest = 0;
    car(k).qclosest = car(k).q;
    car(k).dclosest = 0;
    % - userdata
    car(k).userdata = struct;
end

% - create figure
if (dodisplay)
    hCar = UpdateCar(0,road,car,params,[]);
end

pause(1);
% - loop through each time step
nSkip = 2;
sim.t = 0;
counter = 0;
while (1)
    
    % - update time
    sim.t = sim.t+params.dt;
    if (sim.t>params.tmax)
        donetime = sim.t;
        break;
    end
    
    % - update state, input, error, and maximum position error
    [car,donetime] = OneStep(sim,car,road,params);
    if (~isempty(donetime))
        break;
    end
    
    % - update figure
    if (dodisplay&&(counter>=nSkip))
        hCar = UpdateCar(sim.t,road,car,params,hCar);
        counter = 0;
    else
        counter = counter + 1;
    end
    
end
if (dodisplay)
    hCar = UpdateCar(donetime,road,car,params,hCar);
end
%
% %%%%%%%%%%%%%%%%%%

function [car,donetime] = OneStep(sim,car,road,params)
donetime = sim.t;
for k=1:length(car)
    
    if (isnan(car(k).result))
        % - don't quit
        donetime = [];
        % - update sdes
        car(k).sdes = car(k).sdes+(car(k).vdes*params.dt);
        % - get reference
        [car(k).qdes,car(k).wdes] = WhereAmI(car(k).sdes,car(k).vdes,road);
        % - update sclosest
        [car(k).sclosest,car(k).dclosest,car(k).qclosest] = GetClosestPoint(car(k).q,car(k).sclosest,road);
        % - error in body-fixed reference frame
        e = GetError(car(k).q,car(k).qdes);
        % - check for crash
        if (car(k).dclosest > 0.5*params.roadwidth)
            car(k).eventtime = sim.t;
            car(k).result = 0;
            continue;
        end
        % - check for win
        if (car(k).sclosest >= road.s(end))
            car(k).eventtime = sim.t;
            car(k).result = 1;
            continue;
        end
        % - right and left wheel speeds
        wR = car(k).q(4);
        wL = car(k).q(5);
        % - run control loop
        [tauR,tauL,car(k).vdes,car(k).userdata] = car(k).GetInput(e(1:2),wR,wL,car(k).vdes,car(k).wdes,car(k).userdata,params);
        % - bound the right and left commanded wheel torques
        tauR = BoundTorque(tauR,params.taumax);
        tauL = BoundTorque(tauL,params.taumax);
        % - integrate
        [t,q] = ode45(@(t,q) fq(t,q,tauR,tauL,params),[0 params.dt],car(k).q);
        q = q(end,:)';
        % - package
        car(k).q = q;
        car(k).tauR = tauR;
        car(k).tauL = tauL;
        car(k).e = e;
    end
    
end

function students = GetStudents
addpath('students');
listing = dir('students');
fprintf(1,'\nFINDING CONTROL SYSTEMS\n');
fprintf(1,' filename : name\n');
nstudents = 0;
for i=1:length(listing)
    if (~listing(i).isdir)
        [tmp1,student.filename,tmp2] = fileparts(listing(i).name);
        if (~isempty(student.filename))
            nstudents = nstudents + 1;
            data = textscan(student.filename,'hw9_%s');
            student.name = data{1}{1};
            students(nstudents) = student;
            fprintf(1,'  %s : %s\n',student.filename,student.name);
        end
    end
end
fprintf(1,'\n\n');

function [sclosest,dclosest,qclosest] = GetClosestPoint(q,s,road)
ds_res = 1e-2;
ds_max = 0.25;
sclosest = nan;
qclosest = nan;
dclosest = inf;
smin = max(0,s-ds_max);
smax = min(road.s(end),s+ds_max);
s = linspace(smin,smax,1+ceil((smax-smin)/ds_res));
for i=1:length(s)
    qroad = WhereAmI(s(i),1,road);
    d = norm(qroad(1:2)-q(1:2));
    if (d<dclosest)
        sclosest = s(i);
        dclosest = d;
        qclosest = qroad;
    end
end

function e = GetError(q,qdes)
e = [RotMat(q(3))'*(q(1:2)-qdes(1:2)); q(3)-qdes(3)];

function tau = BoundTorque(tau,taumax)
if (tau>taumax)
    tau = taumax;
elseif (tau<-taumax)
    tau = -taumax;
end

function dqdt = fq(t,q,tauR,tauL,params)
wR = q(4);
wL = q(5);
v = (params.r/2)*(wR+wL);
w = (params.r/(2*params.b))*(wR-wL);
dqdt = [v*cos(q(3)); v*sin(q(3)); w; params.ktau*tauR; params.ktau*tauL];

function R = RotMat(h)
R = [cos(h) -sin(h); sin(h) cos(h)];

function p = RigidBodyTransform(p,R,d)
p = (R*p)+repmat(d,1,size(p,2));

function h=DrawRoad(road,params,ds)
s = 0;
q = road.q(:,1);
w = road.w(:,2);
while (s<road.s(end))
    
    s = s+ds;
    [qcur,wcur] = WhereAmI(s,1,road);
    q(:,end+1) = qcur;
    w(:,end+1) = wcur;
    
end
qL = q(1:2,:)+0.5*params.roadwidth*[-sin(q(3,:)); cos(q(3,:))];
qR = q(1:2,:)-0.5*params.roadwidth*[-sin(q(3,:)); cos(q(3,:))];
qroad = [qR fliplr(qL) qR(:,1)];
h.fill = patch(qroad(1,:),qroad(2,:),'y','linestyle','none');
h.borderL = plot(qL(1,:),qL(2,:),'k-','linewidth',2);
h.borderR = plot(qR(1,:),qR(2,:),'k-','linewidth',2);
h.center = plot(q(1,:),q(2,:),'-','linewidth',1,'color',0.75*[1 1 1]);

function hCar = UpdateCar(t,road,car,params,hCar)
if isempty(hCar)
    
    % - create figure
    scrsz = get(0,'screensize');
    w = floor((0.9*scrsz(3))/5)*5;
    h = 0.6*w;
	hCar.fig = figure(...
        'position',[10 10 w h],...
        'dockcontrols','off','menubar','none','toolbar','none',...
        'resize','off','renderer','painters');
    
    % - create axes
    hCar.axes = axes(...
        'fontname','fixedwidth',...
        'fontsize',18,'position',[0.03 0.05 0.5 0.9]);
    axis equal;
    hold on;
    w = 1.25;
    h = ((0.9*0.6)/0.5)*w;
    hCar.w = w;
    hCar.h = h;
    set(hCar.axes,'xlim',[-w w],'ylim',[-h h]);
    set(hCar.axes,'xtick',[],'ytick',[],'box','on');
    clear w h scrsz
    
    % - draw road
    DrawRoad(road,params,5e-2);
    
    % - get parameters
    b = params.b;
    r = params.r;
    w = 0.2*r;
    
    % - template car
    hCar.pAxle = GetRectangle(r/10,b);
    hCar.pBody = GetRectangle(1.5*r,0.7*b);
    pWheel = GetRectangle(r,w);
    hCar.pLWheel = [pWheel(1,:); pWheel(2,:)-b];
    hCar.pRWheel = [pWheel(1,:); pWheel(2,:)+b];
    
    % - specific cars
    for k=1:length(car)
        
        R = RotMat(car(k).q(3));
        d = car(k).q(1:2);
        
        p = RigidBodyTransform(hCar.pAxle,R,d);
        hCar.car(k).axle = patch(p(1,:),p(2,:),'k');
        p = RigidBodyTransform(hCar.pBody,R,d);
        hCar.car(k).body = patch(p(1,:),p(2,:),car(k).color);
        p = RigidBodyTransform(hCar.pLWheel,R,d);
        hCar.car(k).leftwheel = patch(p(1,:),p(2,:),car(k).color);
        p = RigidBodyTransform(hCar.pRWheel,R,d);
        hCar.car(k).rightwheel = patch(p(1,:),p(2,:),car(k).color);
        
    end
    
    for k=1:length(car)
        R = RotMat(car(k).q(3));
        d = car(k).q(1:2);
        hCar.car(k).text = text(d(1),d(2),car(k).name,'interpreter','none','fontsize',12);
    end 
                                 
    % - desired trajectory
    for k=1:length(car)
        hCar.qdes(k) = plot(car(k).qdes(1),car(k).qdes(2),'.','markersize',36,'color',car(k).color);
        hCar.qclosest(k) = plot(car(k).qclosest(1),car(k).qclosest(2),'o','markersize',18,'color',car(k).color);
    end
    
    % - title (elapsed time)
    hCar.title = title(sprintf('t = %8.2f',t));
    
    % - scoreboard axes
    hCar.scoreboard = axes(...
        'fontname','fixedwidth',...
        'fontsize',14,'position',[0.97-0.41 0.05 0.41 0.9]);
    set(hCar.scoreboard,'xtick',[],'ytick',[],'box','on');
    
    % - scores
    fSize = 12;
    text(0.025,0.96,'RACING','fontsize',1.5*fSize);
    text(0.375,0.96,'CRASHED','fontsize',1.5*fSize);
    text(0.725,0.96,'FINISHED','fontsize',1.5*fSize);
    for k=1:length(car)
        hCar.score(k) = text(0.05,0.95-k*0.03,sprintf('%4.1f : %s\n',car(k).sclosest,car(k).name),'fontsize',fSize,'interpreter','none','color','k');
        
        xlim = [inf -inf];
        ylim = [inf -inf];
        indcur = k;
        if (car(indcur).q(1)-2*params.roadwidth<xlim(1))
            xlim(1) = car(indcur).q(1)-2*params.roadwidth;
        end
        if (car(indcur).q(1)+2*params.roadwidth>xlim(2))
            xlim(2) = car(indcur).q(1)+2*params.roadwidth;
        end
        if (car(indcur).q(2)-2*params.roadwidth<ylim(1))
            ylim(1) = car(indcur).q(2)-2*params.roadwidth;
        end
        if (car(indcur).q(2)+2*params.roadwidth>ylim(2))
            ylim(2) = car(indcur).q(2)+2*params.roadwidth;
        end
    end
    set(hCar.axes,'xlim',xlim,'ylim',ylim);
    
else
    
    % - specific cars
    for k=1:length(car)
        
        R = RotMat(car(k).q(3));
        d = car(k).q(1:2);
        
        p = RigidBodyTransform(hCar.pAxle,R,d);
        set(hCar.car(k).axle,'xdata',p(1,:),'ydata',p(2,:));
        p = RigidBodyTransform(hCar.pBody,R,d);
        set(hCar.car(k).body,'xdata',p(1,:),'ydata',p(2,:));
        p = RigidBodyTransform(hCar.pLWheel,R,d);
        set(hCar.car(k).leftwheel,'xdata',p(1,:),'ydata',p(2,:));
        p = RigidBodyTransform(hCar.pRWheel,R,d);
        set(hCar.car(k).rightwheel,'xdata',p(1,:),'ydata',p(2,:));
        
        set(hCar.car(k).text,'position',d);
        
    end
    
    for k=1:length(car)
        set(hCar.qdes(k),'xdata',car(k).qdes(1),'ydata',car(k).qdes(2));
        set(hCar.qclosest(k),'xdata',car(k).qclosest(1),'ydata',car(k).qclosest(2));
    end
    
    set(hCar.title,'string',sprintf('t = %8.2f',t));

    % - scores
    fSize = 12;
    
    % -- racing
    K = [];
    S = [];
    for k=1:length(car)
        if isnan(car(k).result)
            K = [K k];
            S = [S car(k).sclosest];
        end
    end
    if (~isempty(K))
        [tmp,J] = sort(S,'descend');
        xlim = [inf -inf];
        ylim = [inf -inf];
        for j=1:length(J)
            set(hCar.score(K(J(j))),'position',[0.05,0.95-j*0.03,0],'string',sprintf('%4.1f : %s\n',car(K(J(j))).sclosest,car(K(J(j))).name));
            
            indcur = K(J(j));
            if (car(indcur).q(1)-2*params.roadwidth<xlim(1))
                xlim(1) = car(indcur).q(1)-2*params.roadwidth;
            end
            if (car(indcur).q(1)+2*params.roadwidth>xlim(2))
                xlim(2) = car(indcur).q(1)+2*params.roadwidth;
            end
            if (car(indcur).q(2)-2*params.roadwidth<ylim(1))
                ylim(1) = car(indcur).q(2)-2*params.roadwidth;
            end
            if (car(indcur).q(2)+2*params.roadwidth>ylim(2))
                ylim(2) = car(indcur).q(2)+2*params.roadwidth;
            end
        end
        set(hCar.axes,'xlim',xlim,'ylim',ylim);
    end
    
    % -- crashed
    K = [];
    S = [];
    for k=1:length(car)
        if (car(k).result==0)
            K = [K k];
            S = [S car(k).sclosest];
        end
    end
    if (~isempty(K))
        [tmp,J] = sort(S,'descend');
        for j=1:length(J)
            set(hCar.score(K(J(j))),'position',[0.4,0.95-j*0.03,0],'string',sprintf('%4.1f : %s\n',car(K(J(j))).sclosest,car(K(J(j))).name));
        end
    end
    
    % -- finished
    K = [];
    S = [];
    for k=1:length(car)
        if (car(k).result==1)
            K = [K k];
            S = [S car(k).eventtime];
        end
    end
    if (~isempty(K))
        [tmp,J] = sort(S,'ascend');
        for j=1:length(J)
            set(hCar.score(K(J(j))),'position',[0.75,0.95-j*0.03,0],'string',sprintf('%4.1f : %s\n',car(K(J(j))).eventtime,car(K(J(j))).name));
        end
    end
end
drawnow;

function p = GetRectangle(w,h)
p = [-w w w -w -w; -h -h h h -h];

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

% Returns the current pose "q" and turning rate "w" on the road, given the
% current arc-length "s" along the road and the current forward speed "v".
function [q,w] = WhereAmI(s,v,road)
if (s>=road.s(end))
    
    q = road.q(:,end);
    w = v*road.w(end);
    
else
    
    i = find(s<road.s,1,'first');
    
    q0 = road.q(:,i-1);
    ds = s-road.s(i-1);
    dh = road.w(i)*ds;
    
    dq = [ds*mysinc(dh/2)*cos(q0(3)+(dh/2));
          ds*mysinc(dh/2)*sin(q0(3)+(dh/2));
          dh];
    q = q0+dq;
    
    w = v*road.w(i);
    
end

% Returns sin(x)/x (MATLAB defines "sinc" slightly differently).
function y=mysinc(x)
y = sinc(x/pi);
