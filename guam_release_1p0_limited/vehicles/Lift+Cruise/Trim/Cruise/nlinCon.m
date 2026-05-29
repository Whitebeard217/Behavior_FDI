function [ c, ceq ] = nlinCon(x)

% Global Variables
global TRIM_POINT % velocity and flight path angle [ V gamma ]
global LPC        % Lift+Cruise vehicle model
global GRAV       % acceleration due to gravity
global RHO        % air density

% Inputs
Tp    = x(1); % pusher prop thrust  
del_e = x(2); % elevator deflection
alpha = x(3); % angle of attack
Tl    = x(4); % leading edge prop thrust
Tt    = x(5); % trailing edge prop thrust

% Trim point
V     = TRIM_POINT(1);
gamma = TRIM_POINT(2);

% state vectors
om = [0; 0; 0];
vB = [V*cos(alpha); 0; V*sin(alpha)];

% set the control inputs
LPC.Tp(1:4) = Tl;
LPC.Tp(5:8) = Tt;
LPC.Tp(9)   = Tp;
LPC.del_e   = del_e;

% calulate aircraft aero
LPC.aero(RHO, vB, om);

% inequality constraints
%  c(x) <= 0
c = [];

% equality constraints
%  ceq(x)  = 0 
ceq(1) = LPC.D+LPC.mass*GRAV*sin(gamma);
ceq(2) = LPC.L-LPC.mass*GRAV*cos(gamma);
ceq(3) = LPC.My;
