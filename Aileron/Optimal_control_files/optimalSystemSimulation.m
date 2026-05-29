function [x,u,pf,tf] = optimalSystemSimulation(A, B, r, x0,t)
%from notes
pt = [2; 0; 2];
[tb,p] = ode45(@RiccatiEq,t, pt);
pf = flipud(p);
tf = -flipud(tb);
b = B(2,:);
k = b/r * pf(:,2:3);
x(:,1) = x0;
u(:,1) = -k(1,:)*x(:,1);
%closed-loop simulation
for i= 1: length(tf)-1
    x(:,i+1)=expm((A -B*k(i,:))*(tf(i+1)-tf(i))) *x(:,i);
    u(:,i+1) = k(i+1,:) * x(:,i+1);
end
end
function pd = RiccatiEq(t,p)
    ad
    q = [q1 q2; q2 q3];
    om = 0;
    del=1;
    b =1;
    pd = [(-2*om^2*p(2))-((b^2*p(2)^2)/r) + q(1,1);
    p(1)-(2*del*om*p(2))-(om^2*p(3))-((b^2*p(2)*p(3))/r);
    2*p(2)-(4*del*om*p(3))-((b^2*p(3)^2)/r) + q(2,2)];
end