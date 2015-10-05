% AE 353 Problem 3 hw 4
%plotting eigenvalues 

A = [0 9;-9 0];
B = [1;0];
R = .0015431

K1 = lqr(A,B,eye(2),1000);
K2= lqr(A,B,eye(2),.0001);
K3 = lqr(A,B,eye(2),.0015431);

Acl1=eig(A-B*K1);
Acl2=eig(A-B*K2);
Acl3=eig(A-B*K3);


X = [9 -9 0 0 0 0];
Y = [-.0224+9i -.0224-9i -315.8429 -9.0146 -15.6766 -15.5013];
scatter(Y,X)
grid on
