% AE 353 HW 6 Problem 3
% Steven Macenski 

%tuning Q,R to find real eigenvalues

A = [0 0 1 0;0 0 0 1;-8 4 -1 .5; 4 -4 .5 -.5];
B = [0;0;0;.5];
C = [0 1 0 0];
Qc = eye(4);
Rc = .9999;
Qo = 9999;
Ro = eye(4);


K = lqr(A,B,Qc,Rc);
L = lqr(A',C',inv(Ro),inv(Qo))';

Acl = [A -B*K; L*C A-L*C-B*K];

eig(Acl)
