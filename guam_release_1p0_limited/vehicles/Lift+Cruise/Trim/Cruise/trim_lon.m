function [y] = trim_lon(input, aircraft, trim_point, grav, density)

% fairly general trimming function for the longitudinal 
% dynamics

% input = [T del_e alpha] 
% aircraft = [aircraft to be trimmed]
% trim_point = [ V gamma ]

% Inputs
T     = input(1);
del_e = input(2);
alpha = input(3);

% Trim point
V     = trim_point(1);
gamma = trim_point(2);

% state vectors
om = [0; 0; 0];
vB = [V*cos(alpha); 0; V*sin(alpha)];

aircraft.Tp(9) = T;
aircraft.del_e = del_e;
aircraft.aero(density, vB, om);

y(1) = aircraft.D+aircraft.mass*grav*sin(gamma);
y(2) = aircraft.L-aircraft.mass*grav*cos(gamma);
y(3) = aircraft.My;
