% AE 353 HW 6 Problem 1
% Steven Macenski 

A = [0 1;0 -.2];
B = [0;.2];
C = [1 0];
Qc = eye(2);
Rc = .9999;
Qo = 9999;
Ro = eye(2);


K = lqr(A,B,Qc,Rc)
L = lqr(A',C',inv(Ro),inv(Qo))'

Acl = [A -B*K; L*C A-L*C-B*K];
Bcl = [B;B];

eig(Acl)
