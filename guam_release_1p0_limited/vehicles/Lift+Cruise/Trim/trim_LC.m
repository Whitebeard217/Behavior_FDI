function [SimOP,Trim,SimOut] = trim_LC(SimIn,target,verbose)
%function [SimOP,Trim] = trim_LC(SimIn,target,verbose)
%
% Trims GenVehicleSim simulation to target conditions.
%
% Inputs:
%  Target is a structure which has some subset of the following fields:
%     eas          - Equivalent air speed (fps)
%     tas          - True airspeed (fps)
%     alpha        - Angle of attack (deg)
%     beta         - Sideslip (deg), defaults to zero
%     gamma        - Flight path angle (deg)
%     gndtrack     - Ground track angle (deg)
%     roll         - Roll angle (deg)
%     pitch        - Pitch angle (deg)
%     yaw          - Heading angle (deg) [0-360)
%     rollrate     - d/dt(phi) (deg/sec), defaults to zero
%     pitchrate    - d/dt(theta) (deg/sec), defaults to zero
%     yawrate      - d/dt(psi) (deg/sec), defaults to zero
%     pdeg         - Angular velocity (deg/sec)
%     qdeg         - Angular velocity (deg/sec)
%     rdeg         - Angular velocity (deg/sec)
%
%   verbose      - 0(quiet), 1(soln disp), or 2(iterations), defaults to 2
%
%Outputs:
%   SimOP - simulation initial conditions, at trim
%   Trim  - trim parameter structure
%
% Unspecified variables are free, or defaulted to the values shown above.
% To force a defaulted variable to be free define it with an empty matrix.
% For example, by default beta=0  but "target.beta=[];" will allow beta
% to be free in searching for a trim condition.
%
% Examples:
%  SimOP=trim_LC(SimIn,struct('eas',95,'gamma',0));
%  [SimOP,Trim]=trim_LC(SimIn,struct('alpha',3,'gamma',0,'yaw',120),1);
%  [SimOP,Trim]=trim_LC(SimIn,struct('tas',100,'gamma',3,'yawrate',10),0);
%  [SimOP,Trim]=trim_LC(SimIn,struct('tas',120*SimIn.Units.knot,'gamma',0));
%

deg = SimIn.Units.deg;
ft  = SimIn.Units.ft;
sec = SimIn.Units.s;
knot = SimIn.Units.knot;

% %Define surface bounds for trim conditions
% %ThrLim  = [ 0; 100];      % Throttle limits (%)
% EleLim  = [-30.0;  30.0] * deg; % Elevator bounds (degrees)
% %StbLim  = [-12.0;  4.0] * deg;  % Stabilizer bounds (degrees)
% AilLim  = [-30;  30] * deg;     % Flaperon limits (degrees)
% %FlpLim  = [-30;  30] * deg;     % Flap limits (degrees)
% RudLim  = [-30;  30] * deg;     % Rudder limits (deg)

% Noisy by default
if ( ~exist('verbose','var') || isempty(verbose) ), verbose=2; end

% Error checking input
if ( isfield(target,'eas') && isfield(target,'tas') )
    error('May specify a eas or tas, but not both');
end

if isfield(target,'alpha')&&isfield(target,'pitch')&&isfield(target,'gamma'),
    error('Lacking free variable, cannot specify target in alpha,pitch and gamma');
end

knownfields={ 'eas','tas','alpha','beta','gamma','gndtrack',...
              'roll','pitch','yaw',...
              'rollrate','pitchrate','yawrate',...
              'pdeg','qdeg','rdeg'};
saywhat=setdiff(fieldnames(target),knownfields);
if ~isempty(saywhat)
    for i=1:length(saywhat),fprintf(1,'Unknown field in target: %s\n',saywhat{i}),end
    error('Unknown fields');
end

% Defaults if not specified
no_beta_target = 0;
if ~isfield(target,'beta')
  % beta not manually specified as target, so set a flag to control trimming
  no_beta_target = 1; 
  target.beta=0;
end
if ~isfield(target,'rollrate'), target.rollrate=0; end
if ~isfield(target,'pitchrate'),target.pitchrate=0; end
if ~isfield(target,'yawrate'),  target.yawrate=0; end

% Remove fields assocated with empty targets (overrides defaults);
fn=fieldnames(target);
for i=1:length(fn)
    if isempty(target.(fn{i}))
        target=rmfield(target,fn{i});
    end
end

% Warn on zero gndtrack|yaw
if isfield(target,'yaw') && target.yaw==0,
    warning('0 degree yaw is at 0/360 discontinuity, optimizer may stall'); pause(1);
end

%open_system('GenVehicleSim');
sys = 'GenVehicleSim';

% Trim time
t   = 0;% This is a point in time, no simulation up to that time.

% Configure sim for trim inputs
%evalin('base','SimIn.Switches.TrimModeOn = 1;');
%evalin('base','SimIn.Switches.LinearizeModeOn = 0;');
SimIn.Switches.TrimModeOn = 1;

SimIn.IC = setupInitialConditions(SimIn, target);

% Wrap signals so that any discontinuities do not occur within the area of
% trimming.
SimIn.IC.track = wrap(SimIn.IC.track,2*pi,-pi);% [-pi pi]
SimIn.IC.psi   = wrap(SimIn.IC.psi  ,2*pi,-pi);% [-pi pi]

SimIn.EOM = setupEOM(SimIn);

SimIn.Trim.Set.Vel_bEh = 'VAB';% Must set Vel_bEh with Vtot, alpha and beta to fix alpha and beta.
%Trim.Set.Vel_bEh = 'VGC';% Must set Vel_bEh with Vtot, gamma and chi to fix gamma and chi.

% Select variables to perturb/target/constrain

%% Set States
SimIn.Trim.States.Vgrnd.x0    = SimIn.IC.Vgrnd;
SimIn.Trim.States.gamma.x0    = SimIn.IC.gamma;
SimIn.Trim.States.track.x0    = SimIn.IC.track;
SimIn.Trim.States.phidot.x0   = SimIn.IC.phidot;
SimIn.Trim.States.thetadot.x0 = SimIn.IC.thetadot;
SimIn.Trim.States.psidot.x0   = SimIn.IC.psidot;
SimIn.Trim.States.phi.x0      = SimIn.IC.phi;
SimIn.Trim.States.theta.x0    = SimIn.IC.theta;
SimIn.Trim.States.psi.x0      = SimIn.IC.psi;
SimIn.Trim.States.alpha.x0    = SimIn.IC.alpha;
SimIn.Trim.States.beta.x0     = SimIn.IC.beta;
SimIn.Trim.States.Vtot.x0     = SimIn.IC.Vtot;

%% Set Outputs
SimIn.Trim.Outputs.Veas.y0         = SimIn.IC.Veas;
SimIn.Trim.Outputs.Vtot.y0         = SimIn.IC.Vtot;
SimIn.Trim.Outputs.alpha.y0        = SimIn.IC.alpha; 
SimIn.Trim.Outputs.beta.y0         = SimIn.IC.beta; 
SimIn.Trim.Outputs.gamma.y0        = SimIn.IC.gamma;
SimIn.Trim.Outputs.gndtrack.y0     = SimIn.IC.track;
SimIn.Trim.Outputs.Asensed_bIb.y0  = SimIn.EOM.Asensed_bIb;
SimIn.Trim.Outputs.Vtotdot.y0      = 0;
SimIn.Trim.Outputs.alphadot.y0     = 0;
SimIn.Trim.Outputs.betadot.y0      = 0;
SimIn.Trim.Outputs.gammadot.y0     = 0;
SimIn.Trim.Outputs.chidot.y0       = SimIn.IC.psidot;
SimIn.Trim.Outputs.VelDtH_bEh.y0   = [0;0;0];
SimIn.Trim.Outputs.phi.y0          = SimIn.IC.phi;
SimIn.Trim.Outputs.theta.y0        = SimIn.IC.theta;
SimIn.Trim.Outputs.psi.y0          = SimIn.IC.psi;

% Controls/Commands Perturbed
SimIn.Trim.Inputs.engines.perturb = boolean([0 0 0 0 0 0 0 0 1]);
SimIn.Trim.Inputs.surfaces.perturb = boolean([0 1 1 1]); % surface order is [flap ail elev rud]
% may want to set Aircraft specific limits and initial guesses here
SimIn.Trim.Inputs.engines.u0(9) = 500;

% Outputs targeted
SimIn.Trim.Outputs.gamma.target = true;
SimIn.Trim.Outputs.Vtot.target  = true;
SimIn.Trim.Outputs.gndtrack.target = true;
SimIn.Trim.Outputs.phi.target = true;

if no_beta_target == 1
  fprintf(1,'no beta specified\n');
  SimIn.Trim.Outputs.Asensed_bIb.target = [false;true;false]; % ay = 0
  SimIn.Trim.Outputs.Asensed_bIb.y0(2) = 0;
else
  fprintf(1,'beta specified\n');
  SimIn.Trim.Outputs.beta.target  = true; % set beta as target, and let ay float
end

% Derived-States Perturbed

SimIn.Trim.States.Vgrnd.perturb = true;
SimIn.Trim.States.alpha.perturb = true;
SimIn.Trim.States.beta.perturb = true;

SimIn.Trim.States.phi.perturb = true;
SimIn.Trim.States.theta.perturb = true;
SimIn.Trim.States.psi.perturb = true;

SimIn.Trim.States.phidot.perturb = true;
SimIn.Trim.States.thetadot.perturb = true;
SimIn.Trim.States.psidot.perturb = true;

% Derivatives are treated as equality constraints
SimIn.Trim.Derivatives.VelDtB_bEb.constrain([1 2 3]) = true;% udot;vdot;wdot
SimIn.Trim.Derivatives.OmgDtB_BHb.constrain([1 2 3]) = true;% pdot;qdot;rdot
SimIn.Trim.Derivatives.EDt.constrain([1 2]) = true;% phidot;thetadot
%SimIn.Trim.Derivatives.EDt.constrain([1 2 3]) = true;% phidot;thetadot;psidot

%SimIn.Trim.Derivatives.Vtotdot.constrain = true;
%SimIn.Trim.Derivatives.gammadot.constrain = true;
%SimIn.Trim.Derivatives.chidot.constrain   = true;
%SimIn.Trim.Derivatives.alphadot.constrain = true;
%SimIn.Trim.Derivatives.betadot.constrain  = true;

%% Trim function
[x0,u0,y0,dx0,ix,iu,iy,idx,xulb,xuub] = trimStruct2Array(SimIn.Trim);
assignin('base','SimIn',SimIn);

% disable warnings about missing 'From' for 'Goto'
id = 'Simulink:blocks:MatchingFromNotFound';
warning('off', id);

% Setup optimization parameters
para=[];
if (verbose==2), para(1)=1; end    % Verbose display
para(2)=1e-10; % Convergence criteria
para(7)=1;    % Update Hessian
%para(13)=6;   % 6 equality constraints d/dt[uvw;pqr] = 0
para(13)=length(idx)+2*length(iy);   % 6 equality constraints d/dt[uvw;pqr] = 0
%para(14)=2500; % Iteration limit
para(14)=150; % Iteration limit

fun = @(xul) trimEOM_LC(xul,sys,t,x0,u0,y0,dx0,ix,iu,iy,idx,SimIn,SimIn.Trim);% Trim function for EOM
[x,u,y,dx,f,g,para,aux] = trimsim(sys,fun,xulb,xuub,...
    x0,u0,y0,ix,iu,iy,dx0,idx,...
    para);

% re-enable warnings about missing 'From' for 'Goto'
id = 'Simulink:blocks:MatchingFromNotFound';
warning('on', id);

  % Reset trim mode to off
SimIn.Switches.TrimModeOn = 0;
assignin('base','SimIn',SimIn);
%evalin('base','SimIn.Switches.TrimModeOn = 0;');

SimOut = aux;
% Map psuedo-states to actual states
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

SimIn.IC.Veas  = y(1);
SimIn.IC.Vtot  = y(2);
SimIn.IC.alpha = y(3);
SimIn.IC.beta  = y(4);
SimIn.IC.gamma = y(5);
SimIn.IC.track = y(6);

SimIn.IC.VelDtB_bEb  = dx(1:3);
SimIn.IC.OmegDtB_BHb = dx(4:6);

SimIn.IC.VelDtI_bIi  = SimOut.Accel_bIi;
SimIn.IC.OmegDtI_BIb = SimOut.OmegDtI_BIb;

% Update EOM with trimmed IC's
SimIn.EOM = setupEOM(SimIn);

% Update Actuators with trimmed IC's
SimIn.Act = setupActuators(SimIn);

mba = QgetEulers(SimIn.EOM.Q_n2b,132);
SimIn.IC.mu    = mba(1);

SimOP = SimIn.IC;
Trim = SimIn.Trim; % assign trim to keep the output the same
assignin('base','SimIn',SimIn);

% Print results to screen
%% Outputs
if (verbose)
  % output specified targets
  fprintf(1,'\nTargets:\n');
  fprintf(1,'  ax: %+5.4e \tay: %+5.4e \taz: %+5.4e\n',[SimOut.ax; SimOut.ay; SimOut.az]);
  fn = fieldnames(target);
  fn_len = length(fn);
  offset = 0;
  max_fn_per_row = 4;
  if (mod(fn_len, max_fn_per_row) > 0)
    offset = 1;
  end
  fn_rows = floor(fn_len / max_fn_per_row) + offset;
  fn_count = 0;
  for i=1:fn_rows
    fprintf(1,'  ');
    if ((fn_count + max_fn_per_row) > fn_len)
      num_fn_per_row = fn_len - fn_count;
    else
      num_fn_per_row = max_fn_per_row;
    end
    for j=1:num_fn_per_row
      fn_count = fn_count + 1;
      fprintf(1,'%s: %+5.4e \t', fn{fn_count}, target.(fn{fn_count}));
    end
    fprintf(1,'\n');
  end
  
  % psuedo-states
  fprintf(1,'\nStates:\n');
  fprintf(1,'  Vgrnd (fps): %+5.4e \tphidot   (deg/s): %+5.4e \tphi   (deg): %+5.4e\n',[SimIn.IC.Vgrnd; [SimIn.IC.phidot;   SimIn.IC.phi  ]*180/pi]);
  fprintf(1,'  chi   (deg): %+5.4e \tthetadot (deg/s): %+5.4e \ttheta (deg): %+5.4e\n',[SimIn.IC.track;  SimIn.IC.thetadot; SimIn.IC.theta]*180/pi );
  fprintf(1,'  gamma (deg): %+5.4e \tpsidot   (deg/s): %+5.4e \tpsi   (deg): %+5.4e\n',[SimIn.IC.gamma;  SimIn.IC.psidot;   SimIn.IC.psi  ]*180/pi );

  % state derivatives
  fprintf(1,'\nState Derivatives:\n');
  fprintf(1,'  VelDtB_bEb(1) (fps2): %+5.4e \tOmegDtB_BHb(1) (deg/s2): %+5.4e\n',[SimIn.IC.VelDtB_bEb(1); SimIn.IC.OmegDtB_BHb(1)*180/pi]);
  fprintf(1,'  VelDtB_bEb(2) (fps2): %+5.4e \tOmegDtB_BHb(2) (deg/s2): %+5.4e\n',[SimIn.IC.VelDtB_bEb(2); SimIn.IC.OmegDtB_BHb(2)*180/pi]);
  fprintf(1,'  VelDtB_bEb(3) (fps2): %+5.4e \tOmegDtB_BHb(3) (deg/s2): %+5.4e\n',[SimIn.IC.VelDtB_bEb(3); SimIn.IC.OmegDtB_BHb(3)*180/pi]);
  fprintf(1,'  Vtotdot       (fps2): %+5.4e \tVeasdot        (fps2)  : %+5.4e\n',[SimOut.Vtotdot        ; SimOut.Veasdot]);
  fprintf(1,'  alphadot      (dps) : %+5.4e \tgammadot       (deg/s2): %+5.4e\n',[SimOut.alphadot*180/pi; SimOut.gammadot*180/pi]);
  fprintf(1,'  betadot       (dps) : %+5.4e \tchidot         (deg/s2): %+5.4e\n',[SimOut.betadot *180/pi; SimOut.chidot  *180/pi]);

  % simulation outputs
  fprintf(1,'\nOutputs:\n');
  fprintf(1,'  Vtot  (fps): %+5.4e \tVeas  (fps): %+5.4e\n',[SimIn.IC.Vtot; SimIn.IC.Veas]);
  fprintf(1,'  alpha (deg): %+5.4e \tbeta  (deg): %+5.4e\n',[SimIn.IC.alpha*180/pi; SimIn.IC.beta*180/pi]);
  fprintf(1,'  gamma (deg): %+5.4e \ttrack (deg): %+5.4e\n',[SimIn.IC.gamma*180/pi; SimIn.IC.track*180/pi]);

  % control surface positions
  fprintf(1,'\nControl Surfaces:\n');
  fprintf(1,'  Flaperon (deg): [%+5.4e %+5.4e]\tElevator (deg): [%+5.4e %+5.4e]\tRudder (deg): %+5.4e\n',...
          [SimIn.IC.bias.Surfaces(1:2)*180/pi; SimIn.IC.bias.Surfaces(3:4)*180/pi; SimIn.IC.bias.Surfaces(5)*180/pi]);

  % engine rotor speeds
  fprintf(1,'\nPropulsion:\n');
  fprintf(1,'  lift rotor speeds : [%5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f]\ttail propeller speed: %5.2f\t\n\n',...
          SimIn.IC.bias.Engines);
end
