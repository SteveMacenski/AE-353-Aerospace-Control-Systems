%AE 353 hw 6 problem 2
% Steven Macenski 

A = [0 9;-9 0];
B = [1;0];
C = [0 1];
Qc = eye(2);
Rc = .9999;
Qo = 9999;
Ro = eye(2);


K = lqr(A,B,Qc,Rc);
L = lqr(A',C',inv(Ro),inv(Qo))';

Acl = [A -B*K; L*C A-L*C-B*K];
Bcl = [B;B];

eig(Acl)
