function [x,u,Pinf,tf] = subOptimalSimulation(A, B, r, x0)
ad
q = [q1 q2; q2 q3];
p2 = sqrt(q1*r);
p1 = sqrt(q1*q3 + 2*q1*p2);
p3 = sqrt(q3*r + 2*p2);
tf = [0:0.01:2];
Pinf = [p1 p2; p2 p3];
Kinf = 1/r * (B.' * Pinf);
x(:,1) = x0;
u(:,1) = Kinf*x(:,1);
%closed-loop simulation
for i= 1: length(tf)-1
x(:,i+1)=expm((A -B*Kinf)*(tf(i+1)-tf(i))) *x(:,i);
u(:,i+1) = Kinf * x(:,i+1);
end
end