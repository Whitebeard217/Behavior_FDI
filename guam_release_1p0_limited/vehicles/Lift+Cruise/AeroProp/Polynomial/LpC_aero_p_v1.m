function [X,Y,Z,L,M,N,Validity] = LpC_aero_p_v1(x,n,d,blending_method,c)
% LpC_aero - Implements the Lift+Cruise aero model obtained from DOE tests
%
% DESCRIPTION: 
%   This script runs the aerodynamic model obtained from CFD results for
%   the Lift+Cruise vehicle.
%
% INPUT: 
%   x - state vector containing body-axis velocity and body-axis angular 
%       rates, x=[u,v,w,p,q,r] in ft/s and rad/s
%   n - vector containing propeller speed values, n=[n1,n2...] in rad/s
%   d - vector contol surface positions, n=[d1,d2...] in rad
%   blending_method - method of combining overlapping models
%         1=linear
%         2=smooth "sigmoid-like" blending (continuous and differentiable)
%   c - SimIn.Units
%
% OUTPUT:
%   X - x body axis aerodynamic force, lbf
%   Y - y body axis aerodynamic force, lbf
%   Z - z body axis aerodynamic force, lbf
%   L - aerodynamic rolling moment, ft-lbf
%   M - aerodynamic pitching moment, ft-lbf
%   N - aerodynamic yawing moment, ft-lbf
%   Validity - 
%     invalid_speed - flag indicating invalid speed
%     invalid_prop_speed - flag indiating invalid prop speeds
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   NASA Langley Research Center
%   Email: benjamin.m.simmons@nasa.gov
%
% HISTORY:
%   February 12, 2020 - Created and debugged, BMS
%   May 6, 2020 - Added interpolation between new polynomial models, BMS
%

% extract states
u_fps=x(1);
v_fps=x(2);
w_fps=x(3);

p_rps=x(4);
q_rps=x(5);
r_rps=x(6);

% V_fps=sqrt(u_fps^2+v_fps^2+w_fps^2);
% aoa_rad=atan(w_fps/u_fps);
% aos_rad=asin(v_fps/V_fps);

% conversion constants
r2d=1/c.deg;
fps2kts=1/c.knot;
radps2rpm=60/(2*pi);

% Input: facs
facs=zeros(17,1);

u_kts=u_fps*fps2kts;   % x body-axis velocity [kts]
facs(1)=u_kts;         % x body-axis velocity [kts]
facs(2)=v_fps*fps2kts; % y body-axis velocity [kts]
facs(3)=w_fps*fps2kts; % z body-axis velocity [kts]

facs(4)=d(1)*r2d; % Left Aileron [deg]
facs(5)=d(2)*r2d; % Right Aileron [deg]
facs(6)=d(3)*r2d; % Left Elevator [deg]
facs(7)=d(4)*r2d; % Right Elevator [deg]
facs(8)=d(5)*r2d; % Rudder [deg]

facs(9)= n(1)*radps2rpm; % Engine 1 Speed [RPM]
facs(10)=n(5)*radps2rpm; % Engine 2 Speed [RPM]
facs(11)=n(2)*radps2rpm; % Engine 3 Speed [RPM]
facs(12)=n(6)*radps2rpm; % Engine 4 Speed [RPM]
facs(13)=n(3)*radps2rpm; % Engine 5 Speed [RPM]
facs(14)=n(7)*radps2rpm; % Engine 6 Speed [RPM]
facs(15)=n(4)*radps2rpm; % Engine 7 Speed [RPM]
facs(16)=n(8)*radps2rpm; % Engine 8 Speed [RPM]
facs(17)=n(9)*radps2rpm; % Engine 9 Speed [RPM]

[AeroModel, Validity]=LpC_interp_p_v1(u_kts,facs,blending_method);
% Output: AeroModel = [Fnormal Faxial Mpitch Fside Mroll Myaw Flift Fdrag];

% body-axis aerodynamic forces
X=-1*AeroModel(2);
Y=AeroModel(4);
Z=-1*AeroModel(1);

% body-axis aerodynamic moments
L=AeroModel(5);
M=AeroModel(3);
N=AeroModel(6);

end
