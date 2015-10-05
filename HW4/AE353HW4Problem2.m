% AE 353 Problem 2 HW 4

A = [0 1;0 -.2];
B = [0;.2];
C = [1 0];

Q = [100 0; 0 1];
R = [.1];

K = lqr(A,B,Q,R);
Kref = -1/(C*(A-B*K)*B);

S = eig(A-B*K);

