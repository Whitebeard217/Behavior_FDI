function [ ceq ] = trim_eval(x, LPC, grav, rho)

% Inputs
V     = x(1);
gamma = x(2);
Tp    = x(3); % pusher prop thrust  
del_e = x(4); % elevator deflection
alpha = x(5); % angle of attack
Tl    = x(6); % leading edge prop thrust
Tt    = x(7); % trailing edge prop thrust

% state vectors
om = [0; 0; 0];
vB = [V*cos(alpha); 0; V*sin(alpha)];

% set the control inputs
LPC.Tp(1:4) = Tl;
LPC.Tp(5:8) = Tt;
LPC.Tp(9)   = Tp;
LPC.del_e   = del_e;

% calulate aircraft aero
LPC.aero(rho, vB, om);

% equality constraints
%  ceq(x)  = 0 
ceq(1) = LPC.D+LPC.mass*grav*sin(gamma);
ceq(2) = LPC.L-LPC.mass*grav*cos(gamma);
ceq(3) = LPC.My;
