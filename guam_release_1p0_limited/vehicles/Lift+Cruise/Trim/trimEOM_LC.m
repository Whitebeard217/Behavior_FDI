function [f,g,y,dx,aux] = trimEOM_LC(xul,sys,t,x0,u0,y0,dx0,ix,iu,iy,idx,SimIn,Trim)
% 
% 

% eheim 20160311
% $Id: trimEOM_LC.m 2584 2017-08-02 21:11:20Z bbacon $

% Environment: Flat Earth conditions set
if any(isnan(xul))
    xul
end

nx = length(x0(ix));% Number of states/pseudo-states
nu = length(u0(iu));

% Populate default states and controls
x = x0;
u = u0;

% Update state and control vector
x(ix)  = xul(1:nx);
u(iu)  = xul(nx+1:end-1);
lambda = xul(end);% Lagrange Multiplier

% Map pseudo-states to actual states
SimIn.IC.Vgrnd    = x( 1);
SimIn.IC.gamma    = x( 2);
SimIn.IC.track    = x( 3);

SimIn.IC.phidot   = x( 4);
SimIn.IC.thetadot = x( 5);
SimIn.IC.psidot   = x( 6);

SimIn.IC.phi      = x( 7);
SimIn.IC.theta    = x( 8);
SimIn.IC.psi      = x( 9);

SimIn.IC.alpha    = x(10);
SimIn.IC.beta     = x(11);
SimIn.IC.Vtot     = x(12);

SimIn.IC.bias.Engines  = u(1:SimIn.Trim.numEngines);
SimIn.IC.bias.Surfaces = SimIn.Trim.surfaceAlloc*u(SimIn.Trim.numEngines+1:SimIn.Trim.numEngines+SimIn.Trim.numSurfaces);


SimIn.EOM = setupEOM(SimIn);

SimIn.IC.Vtot = SimIn.IC.Vgrnd;% NO WINDS

%% Atmosphere
% Needed for Mixer model and Veasdot.

% Call Matlab Atmosphere Function
Atm = ussa76(SimIn.IC.AltMSL/SimIn.Units.m,0/SimIn.Units.K);
rho = Atm.rho*SimIn.Units.kg/SimIn.Units.m^3;
SimIn.IC.qbar = 0.5*rho*SimIn.IC.Vtot^2;

% Map GNC surface IC's to Actuator IC's.
%switch SimIn.Aircraft
%    case {gncAircraft.Foamie, gncAircraft.Woodstock_I}
%        SimIn.Act.IC = SimIn.IC.CS;
%    case {gncAircraft.E1}
%        if Trim.States.DAil.perturb == true,
%        SimIn.IC.CS([3 4]) = SimIn.IC.DAil*[-1 1];
%        end;
%        SimIn.Act.IC = SimIn.IC.CS([4 2 5 6 5 1 3]);
%        SimIn.Prop.Omeg_prop.IC = SimIn.IC.EngineSpeed;
%end

%% Set states by state name
Xo = Simulink.BlockDiagram.getInitialState(sys);
switch SimIn.eomType
  case EOMEnum.STARS
    Xo.signals(strcmp({Xo.signals.stateName},'Omeg_BIb' )).values = SimIn.EOM.Omeg_BIb;
    Xo.signals(strcmp({Xo.signals.stateName},'Q_i2b'    )).values = SimIn.EOM.Q_i2b;
    Xo.signals(strcmp({Xo.signals.stateName},'Vel_bEb'  )).values = SimIn.EOM.Vel_bEb;% BJB
    Xo.signals(strcmp({Xo.signals.stateName},'Pos_bii'  )).values = SimIn.EOM.Pos_bii;
  case EOMEnum.Simple
    Xo.signals(strcmp({Xo.signals.stateName},'EOMStates' )).values = [SimIn.EOM.Vel_bEb; SimIn.EOM.Omeg_BIb; SimIn.EOM.Pos_bii; SimIn.EOM.Q_i2b];
  otherwise
end

switch SimIn.actType
  case ActuatorEnum.FirstOrder
    Xo.signals(strcmp({Xo.signals.stateName},'Actuators')).values = SimIn.Act.Bias;
  case ActuatorEnum.SecondOrder
    Xo.signals(strcmp({Xo.signals.stateName},'ActuatorPosition')).values = SimIn.Act.Bias;
  otherwise
end

switch SimIn.propType
  case PropulsionEnum.FirstOrder
    Xo.signals(strcmp({Xo.signals.stateName},'EngineSpeed')).values = SimIn.Eng.Bias;
  case PropulsionEnum.SecondOrder
    Xo.signals(strcmp({Xo.signals.stateName},'EngineSpeed')).values = SimIn.Eng.Bias;
  otherwise
end


% Get outputs and state derivatives
Y  = feval(sys,t,Xo,u,'outputs');
DX = feval(sys,t,Xo,u,'derivs');
y  = Y.signals(1).values(:);

if any(isnan(y))
    y
end

% Outputs
Veas     = y( 1);
Vtot     = y( 2);
alpha    = y( 3);
beta     = y( 4);
gamma    = y( 5);
track    = y( 6);
ax       = y( 7);
ay       = y( 8);
az       = y( 9);
Vtotdot  = y(10);
alphadot = y(11);
betadot  = y(12);
gammadot = y(13);
chidot   = y(14);
hddot    = -y(17);
%phi      = y(18);
%theta    = y(19);
%psi      = y(20);

switch SimIn.eomType
  case EOMEnum.STARS
    OmegDtI_BIb = DX.signals(strcmp({DX.signals.stateName},'Omeg_BIb')).values; OmegDtI_BIb=OmegDtI_BIb(:);
    VelDtB_bEb = DX.signals(strcmp({DX.signals.stateName},'Vel_bEb' )).values; VelDtB_bEb=VelDtB_bEb(:);% BJB
  case EOMEnum.Simple
    Derivs = DX.signals(strcmp({DX.signals.stateName},'EOMStates')).values;
    OmegDtI_BIb = Derivs(1,4:6); OmegDtI_BIb=OmegDtI_BIb(:);
    VelDtB_bEb = Derivs(1,1:3); VelDtB_bEb=VelDtB_bEb(:);% BJB
  otherwise
end

phi      = SimIn.IC.phi;  % (rad)
theta    = SimIn.IC.theta;% (rad)
Q_i2b    = SimIn.EOM.Q_i2b;
Q_h2b    = Qmult(Qinvert(SimIn.EOM.Q_i2h),SimIn.EOM.Q_i2b);
Omeg_BIb = SimIn.EOM.Omeg_BIb;% (rad/s)
Omeg_BEb = SimIn.EOM.Omeg_BEb;% (rad/s)
Omeg_BHb = SimIn.EOM.Omeg_BHb;% (rad/s)
Omeg_HEb = SimIn.EOM.Omeg_HEb;% (rad/s)
Omeg_EIb = SimIn.EOM.Omeg_EIb;% (rad/s)

% Partial derivative of velocity of vehicle relative to local horizontal 
Vel_bEb    = SimIn.EOM.Vel_bEb;
VelDtI_bIb = VelDtB_bEb + cross(Omeg_BIb,Vel_bEb);  %Flat Earth
VelDtI_bIi = Qtrans(Qinvert(Q_i2b),VelDtI_bIb);

% Partial derivative of angular rate of vehicle relative to local horizontal 
%  QDt_i2e  = 0.5*Qmult(Q_i2e,[0;Omeg_EIe]);
% [OmegDtH_HEh,~] = Omeg_HEh_PD(VelDtI_bIi,Vel_bIi,QDt_i2e,Vel_bIi,Pos_bii,Q_i2e,SimIn);
OmegDtH_HEh = [0;0;0];% Flat Earth Assumption
OmegDtH_HEb = Qtrans(Q_h2b,OmegDtH_HEh);
% OmegDtB_HEb = OmegDtH_HEb - cross(Omeg_BIb - Omeg_EIb, Omeg_HEb);
OmegDtB_HEb = OmegDtH_HEb - cross(Omeg_BEb, Omeg_HEb);
OmegDtB_EIb = -cross(Omeg_BIb, Omeg_EIb);
OmegDtB_BHb = OmegDtI_BIb-(OmegDtB_HEb + OmegDtB_EIb);

% Euler angle derivatives 
EDt = [1 sin(phi)*tan(theta)  cos(phi)*tan(theta);
       0 cos(phi)            -sin(phi); 
       0 sin(phi)/cos(theta)  cos(phi)/cos(theta)]*Omeg_BHb;

% Rate of change of Equivalent Airspeed  
sigma   = Atm.sigma;
Hd      = Atm.Hd*SimIn.Units.m;
Veasdot = sqrt(sigma)*(Vtotdot - 1/2*Vtot*hddot/Hd); 

% Final state/psuedo-state derivatives
dx = [
       VelDtB_bEb
       OmegDtB_BHb
       EDt
       Veasdot
       gammadot
       chidot
       Vtotdot
       alphadot
       betadot
       hddot
%       hdot_eng
];

if any(isnan(dx))
    dx
end

% Form constraints that minimize deviation from intended values.

ix = []';
iu = []';

%gg = [x(ix)-x0(ix);y(iy)-y0(iy);u(iu)-u0(iu)];
gg = [y(iy) - y0(iy)];
g  = [dx(idx)-dx0(idx); gg-lambda; -gg-lambda];

% Objective function is just lambda.
f = lambda;

if any(isnan(g))
    g
end

% Add simulation outputs here:
%aux.AC.Inertial.Accel_bIi = VelDtI_bIi;
%aux.AC.Inertial.OmegDtI_BIb = OmegDtI_BIb;
%aux.AC.WorldRel.gammadot = gammadot;
%aux.AC.WorldRel.chidot = chidot;
%aux.AC.AirRel.Vtotdot = Vtotdot;
%aux.AC.AirRel.alphadot = alphadot;
%aux.AC.AirRel.betadot = betadot;
%aux.AC.AirRel.Veasdot  = Veasdot;
%aux.AC.Atmos.Air.Atm = Atm;
aux.Accel_bIi   = VelDtI_bIi;
aux.OmegDtI_BIb = OmegDtI_BIb;
aux.gammadot = gammadot;
aux.chidot = chidot;
aux.Vtotdot = Vtotdot;
aux.alphadot = alphadot;
aux.betadot = betadot;
aux.Veasdot  = Veasdot;
aux.ax = ax;
aux.ay = ay;
aux.az = az;
