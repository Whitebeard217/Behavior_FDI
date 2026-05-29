function [AeroModel, Validity] = LpC_interp_p(u_kts,facs,blending_method)
% LpC_interp_p - This script interpolates between Lift+Cruise aero models
%
% DESCRIPTION: 
%   This script selects aerodynamic models identified for the Lift+Cruise
%   UAM-class vehicle based on the u velocity component. Models
%   are blended together in ranges of u which overlap.
%
% INPUT: 
%   u_kts - x-body axis velocity [kts]
%   facs  - vector of factors for Lift+Cruise (states & controls)
%           (units are kts, degrees, and RPM)
%           --See the variable definitions in "LpC_aero_p.m"
%   blending_method - method of combining overlapping models
%         1=linear
%         2=smooth "sigmoid-like" blending (continuous and differentiable)
%         3=smooth "sigmoid-like" blending (continuous and twice differentiable)
%
% OUTPUT:
%   AeroModel - matrix of aerodynamic forces [lbf] and moments [ft-lbf]
%               with column order: [Fnormal Faxial Mpitch Fside Mroll Myaw Flift Fdrag];
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
%   May 6, 2020 - Created and debugged, BMS
%   May 12, 2020 - added glider model, BMS
%   June 22, 2020 - added airplane model, BMS
%

% rotor/propeller speed (RPM)
n_rotor_rpm=facs(9:16);
n_prop_rpm=facs(17);

% initialize return variable
AeroModel = zeros(1,8);

invalid_prop_speed = true;

invalid_speed = true;

% check for valid blending method
%% should probably be an enumeration
invalid_blending = false;
if (blending_method ~= 1) && (blending_method ~= 2) && (blending_method ~= 3)
  invalid_blending = true;
end

% Use only Trans-Vertical model
if u_kts>=-5 && u_kts<=20
    % Transition-Vertical Model u=[-5:25] kts
    invalid_speed = false;    

    if min(n_rotor_rpm)>=550 && n_prop_rpm>=750
        % Transition-Vertical model with rotor/propellers
        AeroModel = LC_TransV_17F_Quad_FM_alph0p05_fun(facs);
        invalid_prop_speed = false;
    elseif max(abs(n_rotor_rpm))==0 && n_prop_rpm==0
        % Transition-Vertical Glider model (no rotor/propellers)
        AeroModel = LC_TransV_BareAir8F_Cubic_FM_alph0p05_fun(facs);
        invalid_prop_speed = false;
    end 
 
% Use only Trans-Cruise model
elseif u_kts>=25 && u_kts<50
    % Transition-Cruise Model u=[25:100] kts
    invalid_speed = false;    
    
    if min(n_rotor_rpm)>=550 && n_prop_rpm>=750
         % Transition-Cruise model with rotor/propellers
        AeroModel = LC_TransC_17F_Quad_FM_alph0p05_fun(facs);
        invalid_prop_speed = false;
    elseif max(abs(n_rotor_rpm))==0 && n_prop_rpm==0
        % Transition-Cruise Glider model (no rotor/propellers)
        AeroModel = LC_BareAir_TransC_8F_Cubic_FM_alph0p05_fun(facs);
        invalid_prop_speed = false;
    end 
 
% Use only Cruise (Airplane) model (rotors off, pusher prop on)
elseif u_kts>100 && u_kts<=130
    % Cruise (Airplane) Model u=[50:130] kts
    invalid_speed = false;    
    
    if n_prop_rpm>=750
         % Cruise model with propeller only
        AeroModel = LC_Cruise_9F_Cubic_FM_alph0p05_fun(facs);
        invalid_prop_speed = false;
    elseif n_prop_rpm==0
        AeroModel = LC_BareAirCruise_8F_Cubic_FM_alph0p05_fun(facs);
        invalid_prop_speed = false;
    end 

% Blend models        
else
    % initialize variables
    u1_end = 1;
    u2_start = 0;
    AeroModel_1 = zeros(1,8);
    AeroModel_2 = zeros(1,8);
    
    if u_kts>20 && u_kts<25
        % blend the Trans-V and Trans-C models in overlapping region
        invalid_speed = false;    

        % model 1 high airspeed [kts]
        u1_end=25;

        % model 2 low airspeed [kts]
        u2_start=20;

        if min(n_rotor_rpm)>=550 && n_prop_rpm>=750
            % models with rotors/propeller
            invalid_prop_speed = false;

            % run model 1 (upper factor range)
            AeroModel_1 = LC_TransV_17F_Quad_FM_alph0p05_fun(facs);

            % run model 2 (lower factor range)
            AeroModel_2 = LC_TransC_17F_Quad_FM_alph0p05_fun(facs);
        elseif max(abs(n_rotor_rpm))==0 && n_prop_rpm==0
            % models with no rotors/propeller
            invalid_prop_speed = false;

            % run model 1 (upper factor range)
            AeroModel_1 = LC_TransV_BareAir8F_Cubic_FM_alph0p05_fun(facs);

            % run model 2 (lower factor range)
            AeroModel_2 = LC_BareAir_TransC_8F_Cubic_FM_alph0p05_fun(facs);
        end
        
    elseif u_kts>=50 && u_kts<=100
        % blend the Trans-C and Cruise models in overlapping region
        invalid_speed = false;    

        % model 1 high airspeed [kts]
        u1_end=100;

        % model 2 low airspeed [kts]
        u2_start=50;
        
        if min(n_rotor_rpm)>=550 && n_prop_rpm>=750
            % models with rotors/propeller
            invalid_prop_speed = false;

            % run model 1 (upper factor range) ONLY TRANS-C
            AeroModel_1 = LC_TransC_17F_Quad_FM_alph0p05_fun(facs);

            % run model 2 (lower factor range) ONLY TRANS-C
            AeroModel_2 = LC_TransC_17F_Quad_FM_alph0p05_fun(facs);
        elseif max(abs(n_rotor_rpm))==0 && n_prop_rpm>=750
            % models with propeller only
            invalid_prop_speed = false;

            % run model 1 (upper factor range)  ONLY CRUISE
            AeroModel_1 = LC_Cruise_9F_Cubic_FM_alph0p05_fun(facs);

            % run model 2 (lower factor range)  ONLY CRUISE
            AeroModel_2 = LC_Cruise_9F_Cubic_FM_alph0p05_fun(facs);
        elseif max(abs(n_rotor_rpm))==0 && n_prop_rpm==0
            % models with no rotors/propeller
            invalid_prop_speed = false;

            % run model 1 (upper factor range)
            AeroModel_1 = LC_BareAir_TransC_8F_Cubic_FM_alph0p05_fun(facs);

            % run model 2 (lower factor range)
            AeroModel_2 = LC_BareAirCruise_8F_Cubic_FM_alph0p05_fun(facs);
        end
    end

    % difference between the lower model high end and upper model low end
    Delta_u = u1_end - u2_start;
    % initialize default weighting values
    w1 = 0.0;
    w2 = 0.0;
    if (invalid_speed == false)
      if blending_method==1 % linear weighting
        % linear weights (continuous)
        w1=(u1_end-u_kts)/Delta_u;
        w2=1-w1;
      elseif blending_method==2 % smooth weighting
        % "sigmoid-like" function for smooth blending (continuous and once differentiable)
        w_angle= (u1_end-u_kts)/Delta_u * pi/2;
        w1=sin(w_angle).^2;
        w2=1-w1;
      elseif blending_method==3 % smooth weighting (continuous and twice differentiable)
        % "sigmoid-like" function for smooth blending (Credit: Dan Moerder)
        wx= (u1_end-u_kts)/Delta_u;
        w1=6*wx.^5-15*wx.^4+10*wx.^3;
        w2=1-w1;
      end
    end
    
    % combine weighted results
    AeroModel = w1*AeroModel_1 + w2*AeroModel_2;
end

Validity = [invalid_speed invalid_prop_speed];

%% error statements not needed for these 2 flags due to assertion checks on
%% vehicle model in simulink

%if (invalid_speed == true)
%  error('the aerodynamic model''s validity has been violated')
%end

%if (invalid_prop_speed == true)
%  error('rotor/prop speed is above and below the minimum operational speed')
%end

if (invalid_blending == true)
  error('invalid interpolation method')
end

% Output: AeroModel = [Fnormal Faxial Mpitch Fside Mroll Myaw Flift Fdrag];
%         Validity  = [invalid_speed invalid_prop_speed];    