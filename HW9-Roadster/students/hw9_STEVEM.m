function [tauR,tauL,vdes,userdata] = hw9_STEVEM(e,wR,wL,vdes,wdes,userdata,params)
%
% INPUTS
%
%  e(1,1) is a measurement of the lateral position error
%  e(2,1) is a measurement of the longitudinal position error
%  wR is a measurement of the right wheel speed
%  wL is a measurement of the left wheel speed
%
%  vdes is the current desired forward speed
%  wdes is the current desired turning rate
%
%
% OUTPUTS
%
%  tauR is the right wheel torque
%  tauL is the left wheel torque
%
%  vdes is the new desired forward speed, should you want to change it
%
%  userdata has whatever you put there
%
persistent isFirstTime 
if isempty(isFirstTime)
    isFirstTime = false;
    fprintf(1,'initialize control loop\n');
        
    userdata.A = [0 0 0 1 0;0 0 vdes 0 0;0 0 0 0 1;0 0 0 0 0; 0 0 0 0 0];
    userdata.B = [ 0 0; 0 0; 0 0; 1 0; 0 1];
    userdata.C = [1 0 0 0 0;0 1 0 0 0; 0 0 0 1 0; 0 0 0 0 1];
    
    
    Qc1 = 1e4;
    Qc2 = 1e6;
    Qc3 = 1e4;
    Qc4 = 1e4;
    Qc5 = 1e4;
    
    Qc = [Qc1 0 0 0 0;0 Qc2 0 0 0; 0 0 Qc3 0 0;0 0 0 Qc4 0;0 0 0 0 Qc5];%same size as A
    Rc = eye(2); %number of inputs
    userdata.K = lqr( userdata.A, userdata.B,Qc,Rc);
    
    Qo1 = 1e-2;
    Qo2 = 1e-1;
    Qo3 = 1e-3;
    Qo4 = 1e-5;
    
    Qo = [Qo1 0 0 0;0 Qo2 0 0; 0 0 Qo3 0;0 0 0 Qo4];
    Ro = eye(5);%number states
    userdata.L = lqr(userdata.A', userdata.C',inv(Ro),inv(Qo))';
    
     userdata.xhat =  [0;0;0;0;0];
    vdes = 1.3;

end

v = params.r/2*(wR + wL);
w = params.r/(2*params.b)*(wR-wL);
y = [e(1); e(2); v-vdes; w-wdes];

userdata.xhat =  userdata.xhat + params.dt*((userdata.A-(userdata.B*userdata.K)-(userdata.L*userdata.C))* userdata.xhat + userdata.L*y);
u = - userdata.K* userdata.xhat;

tauR = u(1) / (params.r*params.ktau);
tauL = u(2) / (params.r*params.ktau);



