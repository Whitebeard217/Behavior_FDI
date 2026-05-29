function [CT,CQ]=LpC_PusherProp_Model(Jx)
% LpC_PusherProp_Model - Pusher propeller aerodynamic model for Lift+Cruise
%
% DESCRIPTION: 
%   This script contains the pusher propeller aerodynamics model for the
%   NASA Lift+Cruise aircraft. The modeling methodology for CT and CQ is
%   described below.
% 
% INPUTS:
%   Jx - normal advance ratio:
%        Jx=V*cosd(Alpha)*cosd(Beta)/(n*D)=u/(n*D)
%        where: 
%             V = airspeed [ft/s]
%             u = x body-axis velocity [ft/s]
%             Alpha = angle of attack [deg]
%             Beta = angle of sideslip [deg]
%             n = propeller rotational speed [rev/s]
%             D = propeller diameter [ft]
%
% OUTPUTS:
%   CT - thrust coefficient [CT=T/(rho*n^2*D^4)]
%   CQ - torque coefficient [CQ=Q/(rho*n^2*D^5)]
%
% CALLS:
%   None
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   NASA Langley Research Center
%   Email: benjamin.m.simmons@nasa.gov
%
% HISTORY:
%   February 23, 2021 - Created and debugged, BMS
%
% ***FOR INTERNAL NASA USE ONLY***
%

% Thrust coefficient was identified using the following data (combined):
% - Difference between "Transition Regime" CFD data and "Hover Regime"
%    polynomial model predictions for axial force.
% - Difference between "Cruise Regime" CFD data and the "Glider Regime"
%    polynomial model predictions for axial force.
% These data give a rough indication of the axial force produced by the
% pusher propeller.
%
% The axial force produced by the propeller was normalized by (rho*n^2*D^4)
% to compute the thrust coefficient. Models were then fit using the axial
% component of advance ratio, Jx, as the explanatory variable.
%
% Data used for modeling was duplicated over zero to create a symmetric
% model and the data used for modeling was restricted to J<2.
CT = -2.123873237735236e-01.*Jx.^2 + 3.965946758735335e-01;

% torque coefficient was identified using the following data:
% - Difference between "Cruise Regime" CFD data and the "Glider Regime"
%    polynomial model predictions for rolling moment.
% These data give a rough indication of the rolling moment produced by the
% pusher propeller. The data in the Hover Regime was deemed too noisy to
% use for this analysis.
%
% The rolling moment produced by the propeller was normalized by
% (rho*n^2*D^5) to compute the torque coefficient.
%
% Models were then fit using the axial component of advance ratio, Jx, as
% the explanatory variable.
%
% Data used for modeling was duplicated over zero to create a symmetric
% model and the data used for modeling was restricted to J<2.
CQ = 3.705976385963508e-02.*Jx.^2 -6.599151230379886e-02;

end