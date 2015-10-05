% AE 353 Hw 4 P 1
%with different outputs, find Q,R tp find Kref

R = 1e-6;
C1 = [1 0 0 0];
C2 = [0 1 0 0];
C3 = [-1 1 0 0];

Q1 = C1'*C1
Q2 = C2'*C2
Q3 = C3'*C3

A = [0 0 1 0;0 0 0 1;-8 4 -1 .5;4 -4 .5 -.5];
B = [0; 0; 0; .5];

K1 = lqr(A,B,Q1,R)
K2 = lqr(A,B,Q2,R)
K3 = lqr(A,B,Q3,R)

Kref1 = (-(C1*inv((A-B*K1))*B))^-1;
Kref2 = -1/(C2*inv((A-B*K2))*B);
Kref3 = -1/(C3*inv((A-B*K3))*B);