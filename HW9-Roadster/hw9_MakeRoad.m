function hw9_MakeRoad

% Clean up workspace code made in its entirety by Professor Bretl
clc
clear all
close all

% Define parameters
params.pstraight = 0.15;
params.stot = 120;
params.smax = 5;
params.smin = 1;
params.roadwidth = 2;
params.hmax = 3*pi/4;
params.rmin = 0.6*params.roadwidth;
params.rmax = 2.0*params.roadwidth;
params.maxnumtries = 3;
params.drawres = 5e-2;
params.colres = 1e-1;

% Initialize road
road.s = [0 params.smax];
road.w = [0 0];
road.q = [0 params.smax;0 0;0 0];

% Iterate
while (1)
    [road,res] = ExtendRoad(road,params);
    if (res)
        break;
    end
end
DrawRoad(road,params);
save('testroad.mat','road');
fprintf('road saved to ''testroad.mat''\n');

function [road,res] = ExtendRoad(road,params)
res = 0;

% turning rate
if (rand<params.pstraight)
    w = 0;
else
    r = params.rmin + ((params.rmax-params.rmin)*rand);
    w = sign(rand-0.5)*(1/r);
end

% arc length
ds = params.smin + (params.smax-params.smin)*rand;
if (abs(ds*w)>params.hmax)
    ds = abs(params.hmax/w);
end

% current road segment
road.s(:,end+1) = road.s(:,end)+ds;
road.w(:,end+1) = w;
road.q(:,end+1) = WhereAmI(road.s(:,end),road);

DrawRoad(road,params);

[col,q1,q2] = IsSelfCollision(road,params.colres,params.roadwidth);
% check current road segment for collisions
if col
    road.s = road.s(1:end-1);
    road.w = road.w(1:end-1);
    road.q = road.q(:,1:end-1);
    plot(q1(1),q1(2),'r.',q2(1),q2(2),'b.');
    drawnow;
    return;
else
    % stop if road is long enough
    if (road.s(:,end)>params.stot)
        road.s(:,end)=params.stot;
        road.q(:,end) = WhereAmI(road.s(:,end),road);
        res = 1;
        return;
    end
    
    for i=1:params.maxnumtries
        [road,res] = ExtendRoad(road,params);
        if (res)
            return;
        end
    end
end
road.s = road.s(1:end-1);
road.w = road.w(1:end-1);
road.q = road.q(:,1:end-1);

function h=DrawRoad(road,params)
clf;
axis equal;
hold on;
axis(10*[-1 1 -1 1]);
s = 0;
q = road.q(:,1);
while (s<road.s(end))
    
    s = s+params.drawres;
    qcur = WhereAmI(s,road);
    q(:,end+1) = qcur;
    
end
qL = q(1:2,:)+0.5*params.roadwidth*[-sin(q(3,:)); cos(q(3,:))];
qR = q(1:2,:)-0.5*params.roadwidth*[-sin(q(3,:)); cos(q(3,:))];
qroad = [qR fliplr(qL) qR(:,1)];
h = patch(qroad(1,:),qroad(2,:),'y');
w = max(max(abs(qroad(1,:))),max(abs(qroad(2,:))));
axis(w*[-1 1 -1 1]);
title(sprintf('length = %5.1f / %5.1f',road.s(end),params.stot));
drawnow;


function [col,q1,q2] = IsSelfCollision(road,res,r)
col=0;
q1=[];
q2=[];
ds = road.s(end)-road.s(end-1);
s = linspace(road.s(end-1),road.s(end),1+ceil(ds/res));
scheck = linspace(0,road.s(end-1),1+ceil(road.s(end-1)/res));
if isempty(scheck)
    return
end
qcheck = inf(3,size(scheck,2));
for i=1:length(scheck)
    qcheck(:,i) = WhereAmI(scheck(i),road);
end
for i=1:length(s)
    q2 = qcheck(1:2,:);
    q1 = WhereAmI(s(i),road);
    q1 = q1(1:2);
    
    while (norm(q1-q2(:,end))<r)
        q2 = q2(:,1:end-1);
        if (isempty(q2))
            break;
        end
    end
    if (isempty(q2))
        break;
    end
    
    
    
    d = sqrt(sum((repmat(q1,1,size(q2,2))-q2).^2,1));
    j = find(d<r,1);
    if ~isempty(j)
        col=1;
        q2 = q2(:,j);
        return;
    end
    
end


% Returns the current pose "q" and turning rate "w" on the road, given the
% current arc-length "s" along the road and the current forward speed "v".
function q = WhereAmI(s,road)
if (s>road.s(end))
    
    q = road.q(:,end);
    
else
    
    i = find(s<=road.s,1,'first');
    if (i==1)
        i=2;
    end
    
    q0 = road.q(:,i-1);
    ds = s-road.s(i-1);
    dh = road.w(i)*ds;
    
    dq = [ds*mysinc(dh/2)*cos(q0(3)+(dh/2));
          ds*mysinc(dh/2)*sin(q0(3)+(dh/2));
          dh];
    q = q0+dq;
    
end

% Returns sin(x)/x (MATLAB defines "sinc" slightly differently).
function y=mysinc(x)
y = sinc(x/pi);

