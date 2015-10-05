function [fR,fL,userdata] = hw10_STEVEM(mHorizPos,mVertPos,mAngVel,mBattery,pTo,pFrom,pBattery,userdata,params)
%
% INPUTS
%
%  mHorizPos is a noisy measurement of horizontal position
%  mVertPos is a noisy measurement of vertical position (altitude)
%  mAngVel is a noisy measurement of angular velocity
%
%  mBattery is the percent charge in the battery (from 0 to 100)
%

if userdata.isFirstTime
    userdata.isFirstTime = false;
    fprintf(1,'initialize control loop\n');
    

    userdata.A = [0 0 0 1 0 0;
                  0 0 0 0 1 0;
                  0 0 0 0 0 1;
                  0 0 -params.g 0 0 0;
                  0 0 0 0 0 0;
                  0 0 0 0 0 0];
              
    
  userdata.Bem = [0 0;
                  0 0;
                  0 0;
                  0 0;
                  1/params.m 1/params.m;
                  params.w/params.J -params.w/params.J];
              
  userdata.Bfu = [0 0;
                  0 0;
                  0 0;
                  0 0;
                  1/(params.m+params.mpackage) 1/(params.m+params.mpackage);
                  params.w/(params.J + params.Jpackage) -params.w/(params.J + params.Jpackage)];
    
    userdata.C = [1 0 0 0 0 0;
                  0 1 0 0 0 0;
                  0 0 0 0 0 1];
              
    userdata.d = [0;
                  0;
                  0;
                  0;
                  -params.g;
                  0];
     Rc1 = 1;
     Rc2 = 1;
     
     Qc1 = 1;
     Qc2 = 1;
     Qc3 = 1;
     Qc4 = 1;
     Qc5 = 1;
     Qc6 = 1;
              
    Rc = [Rc1 0;
          0 Rc2];
    
    Qc = [Qc1 0 0 0 0 0;
          0 Qc2 0 0 0 0;
          0 0 Qc3 0 0 0;
          0 0 0 Qc4 0 0;
          0 0 0 0 Qc5 0;
          0 0 0 0 0 Qc5];
      
    userdata.Kem = lqr(userdata.A,userdata.Bem,Qc,Rc);
    userdata.Kfu = lqr(userdata.A,userdata.Bfu,Qc,Rc);
    
    Ro1 = 1;
    Ro2 = 1;
    Ro3 = 1;
    Ro4 = 1;
    Ro5 = 1;
    Ro6 = 1;
    
    Qo1 = 1;
    Qo2 = 1;
    Qo3 = 1;
    
    Ro = [Ro1 0 0 0 0 0;
          0 Ro2 0 0 0 0;
          0 0 Ro3 0 0 0;
          0 0 0 Ro4 0 0;
          0 0 0 0 Ro5 0;
          0 0 0 0 0 Ro6];
      
    Qo = [Qo1 0 0;
          0 Qo2 0;
          0 0 Qo3];
      
    userdata.L = lqr(userdata.A',userdata.C',inv(Ro),inv(Qo))';
    
    userdata.xhat = [params.pstart;0;0;0;0];
    
        userdata.charge = 0;

end

  if (isempty(pTo))
     userdata.B = userdata.Bem;
     userdata.K = userdata.Kem;
     udes = [params.m*params.g/2; params.m*params.g/2];
  else
     userdata.B = userdata.Bfu;
     userdata.K = userdata.Kfu;
     udes = [(params.m + params.mpackage)*params.g/2; (params.m + params.mpackage)*params.g/2];
  end

      
P = pFrom;

if (isempty(pTo))
     P = pFrom;
     userdata.B = userdata.Bem;
     userdata.K = userdata.Kem;
     udes = [params.m*params.g/2; params.m*params.g/2];
  else
     P = pTo;
     userdata.B = userdata.Bfu;
     userdata.K = userdata.Kfu;
     udes = [(params.m + params.mpackage)*params.g/2; (params.m + params.mpackage)*params.g/2];
end
  
userdata.P2 = P;
if (mBattery < 20 || userdata.charge == 1)
    P = pBattery;
    userdata.charge = 1;
    
    if mBattery > 90
        P = userdata.P2;
        userdata.charge = 0;
    end
end

if params.timeremaining < 20;
    userdata.P2 = P;
if (mBattery < 20 || userdata.charge == 1)
    P = pBattery;
    userdata.charge = 1;
    
    if mBattery > 50
        P = userdata.P2;
        userdata.charge = 0;
    end
end
end

xdes = [P; 0; 0; 0; 0];


y = [mHorizPos;mVertPos;mAngVel];

u = -userdata.K*(userdata.xhat - xdes) + udes;
userdata.xhat = userdata.xhat + params.dt*(userdata.A*userdata.xhat + userdata.B*u - userdata.L*(userdata.C*userdata.xhat - y) + userdata.d);

fR = u(1);
fL = u(2);