function [sys_quat,sys_euler,londyn_euler,latdyn_euler] = linearize_LC(SimIn,sysType,Ts)
%
% Linearize GenVehicleSim simulation at the current trim point.
%
% Inputs:
%   SimIn   - structure containing various system defitions, used to
%             determine the selected actuator type
%   sysType - enumeration to control which states are in the linearized
%             system (BasicPlant by default, w/ no actuator states)
%   Ts      - Timestep for generating a discrete linear model (0 default)
%
% Outputs:
%   sys_quat  - 6dof system with Quaternion states
%   sys_euler - 6dof system with Euler angle states
%   londyn_euler - Approximate (4th order) longitudinal dynamics
%   latdyn_euler - Approximate (4th order) lateral dynamics
%
% Based on Linearize_L2FGlider.m file from L2F simulation, as well as
% linmodel from GTM simulation.
%
%% Linearization Snapshot Times
% User is expected to modify as nessesary.
% 
% Modeled after Bart's LinModOnTrajectory.m file. 
% 
% Differences: 
% 1) Uses Sim API information already in SimOut to calculate state
%    derivatives instead of Model API.
% 2) All linearization outputs are from SimOut bus. No longer need a
%    special root level output for linearization.
% 3) Uses updated linearization io type names. 
% 4) Uses Control System Toolbox functions to simplify code.
% 5) Generalizes state transformation
% 6) Preserves SamplingGrid.Time array
% 
% See also LinModOnTrajectoryL2F Linear_Model_Algebraic

%% Parse Input Arguments

% Set optional flags
Args = [];
if ~exist('sysType','var') || isempty(sysType)
  sysType = SysTypeEnum.BasicPlant;
end
if ~exist('Ts','var') || isempty(Ts) || Ts <= 0
  Ts   = 0;
end

% Set time array for linearization(s)
t = 0;%[1:60];

%% Specify the model name
model = 'GenVehicleSim';

% disable warnings about missing 'From' for 'Goto'
id = 'Simulink:blocks:MatchingFromNotFound';
warning('off', id);

%% Run Simulation and create SimOut
sim(model,t(end));

%% Linearize

% Linearization requires SingleTasking mode.
set_param(model,'SolverMode','SingleTasking');%,'ReturnWorkspaceOutputs','on');

% Specify the analysis I/Os
clear io

% Open stabilizing loop in quaternion dynamics
if (SimIn.eomType == EOMEnum.STARS)
  Q_i2b_SL = '/Vehicle Simulation/Vehicle Model/Lift+Cruise/Vehicle EOM/Equations of Motion/STARS/Rigid Body Dynamics/Orientation/Product';
else
  Q_i2b_SL = '/Vehicle Simulation/Vehicle Model/Lift+Cruise/Vehicle EOM/Equations of Motion/Simple/Aircraft 6DOF/Quaternion Stabilization/Product';
end
io = linio([model Q_i2b_SL],1,'loopbreak');

% Inputs
if (sysType == SysTypeEnum.BasicPlant)
  % Basic Plant (no Actuators)
  bn = [model '/Vehicle Simulation/Vehicle Model/Lift+Cruise/Surface and Engine Actuator Dynamics'];% Path to control surface and actuator dynamics block
  io(end+1) = linio(bn,2,'openinput',[],'EngSpeed');  % PropAct (rad/s)
  io(end+1) = linio(bn,1,'openinput',[],'Position');  % SurfAct (rad)
else
  % Full Plant w/ Actuators
  bn = [model '/From1'];% Path to Cmd From block
  % ControlBus elements
  io(end+1) = linio(bn,1,'openinput',[],'EngineCmd');       % (lbf)...for now, until proper engine model implemented
  io(end+1) = linio(bn,1,'openinput',[],'CtrlSurfaceCmd');  % (rad)
end

ni = numel(io);% number of inputs

% Outputs
bn = [model '/Vehicle Simulation/Bus Creator'];% Path to SimOut Bus Creator
% If desired output is in SimOut then simply ouput that element from the
% SimOut Bus.
io(end+1) = linio(bn,1,'output',[],'Vehicle.EOM.AirRelativeData.Vtot'         );% (fps)
io(end+1) = linio(bn,1,'output',[],'Vehicle.EOM.AirRelativeData.alpha'        );% (rad)
io(end+1) = linio(bn,1,'output',[],'Vehicle.EOM.AirRelativeData.beta'         );% (rad)
io(end+1) = linio(bn,1,'output',[],'Vehicle.EOM.InertialData.Vel_bIi'         );% (fps)
io(end+1) = linio(bn,1,'output',[],'Vehicle.EOM.InertialData.Asensed_bIb'     );% (fps2)  Accel_bIi in LC_linmodel
io(end+1) = linio(bn,1,'output',[],'Vehicle.EOM.InertialData.Omeg_BIb'        );% (rad/s)
io(end+1) = linio(bn,1,'output',[],'Vehicle.EOM.WorldRelativeData.LatLonAlt.LatGeod');% (rad)
io(end+1) = linio(bn,1,'output',[],'Vehicle.EOM.WorldRelativeData.LatLonAlt.Lon');% (rad)
io(end+1) = linio(bn,1,'output',[],'Vehicle.EOM.WorldRelativeData.LatLonAlt.AltGeod');% (ft)
io(end+1) = linio(bn,1,'output',[],'Vehicle.EOM.WorldRelativeData.Euler.phi'  );% (rad)
io(end+1) = linio(bn,1,'output',[],'Vehicle.EOM.WorldRelativeData.Euler.theta');% (rad)
io(end+1) = linio(bn,1,'output',[],'Vehicle.EOM.WorldRelativeData.Euler.psi'  );% (rad)

% set linearization ios to model, only perform if you want to insert the io
% points into the Simulink model
%setlinio(model,io)

% Specify the operating point
% Use the snapshot times within SimOut.Time
op = t(t<=floor(SimOut.Time.Data(end)));

% Create the options
opt = linearizeOptions('SampleTime',0,'RateConversionMethod','tustin');
% opt = linearizeOptions('SampleTime',0,'RateConversionMethod','zoh');

% Linearize the model
sys_quat = linearize(model,io,op,opt);
   
%% Set State Ordering and Naming
% From here it is up to the end user to rename/reorder as desired.
StateNames = sys_quat.StateName;
    
% Basic Plant States
switch SimIn.eomType
  case EOMEnum.STARS
    pos_idx   = getIndex(StateNames,'Pos_bii')';
    vel_idx   = getIndex(StateNames,'Vel_bEb')';
    omeg_idx  = getIndex(StateNames,'Omeg_BIb')';
    quat_idx  = getIndex(StateNames,'Q_i2b')';
  case EOMEnum.Simple
    eomIndex = getIndex(StateNames,'EOMStates')';
    vel_idx  = eomIndex(1:3);
    omeg_idx = eomIndex(4:6);
    pos_idx  = eomIndex(7:9);
    quat_idx = eomIndex(10:13);
  otherwise
end

% Actuator states
switch SimIn.actType
  case ActuatorEnum.FirstOrder
    act_idx = getIndex(StateNames,'Actuators')';
  case ActuatorEnum.SecondOrder
    act_idx = getIndex(StateNames,'ActuatorPosition')';
    act_idx = [act_idx getIndex(StateNames,'ActuatorRate')'];
  otherwise
end

% Propulsion states
switch SimIn.propType
  case PropulsionEnum.FirstOrder
    eng_idx = getIndex(StateNames,'EngSpeed')';
  case PropulsionEnum.SecondOrder
    eng_idx = getIndex(StateNames,'EngSpeed')';
    eng_idx = [eng_idx getIndex(StateNames,'EngAccel')'];
  otherwise
end

% Set state names, ignoring Quaternion states for now
if (SimIn.eomType == EOMEnum.STARS)
  StateNames(pos_idx) = {'x';'y';'z'};
else
  StateNames(pos_idx) = {'lat';'lon';'alt'};
end
StateNames(vel_idx) = {'u';'v';'w'};
StateNames(omeg_idx) = {'p';'q';'r'};
if (~isempty(act_idx))
  switch SimIn.actType
    case ActuatorEnum.FirstOrder
      StateNames(act_idx) = {'actAileron';'actFlap';'actElevator';'actRudder'};
    case ActuatorEnum.SecondOrder
      StateNames(act_idx) = {'actAileron';'actFlap';'actElevator';'actRudder';
                             'actAileronRate';'actFlapRate';'actElevatorRate';'actRudderRate'};
    otherwise
  end
end

if (~isempty(eng_idx))
  switch SimIn.propType
    case PropulsionEnum.FirstOrder
      StateNames(eng_idx) = {'EngSpeed1';'EngSpeed2';'EngSpeed3';'EngSpeed4';'EngSpeed5';'EngSpeed6';'EngSpeed7';'EngSpeed8';'EngSpeed9'};
    case PropulsionEnum.SecondOrder
      StateNames(eng_idx) = {'EngSpeed1';'EngSpeed2';'EngSpeed3';'EngSpeed4';'EngSpeed5';'EngSpeed6';'EngSpeed7';'EngSpeed8';'EngSpeed9'
                             'EngAccel1';'EngAccel2';'EngAccel3';'EngAccel4';'EngAccel5';'EngAccel6';'EngAccel7';'EngAccel8';'EngAccel9'};
    otherwise
  end
end

sys_quat.StateName = StateNames;

%% Plant with Actuator states

% reorder the states to desired order
sys_quat = xperm(sys_quat,[vel_idx omeg_idx pos_idx quat_idx act_idx]);

% Similar logic below needed if not repositioning linio inputs above
%  - below logic based on first order actuator model
%{
if (sysType == SysTypeEnum.BasicPlant)
  % preserve re-ordered state names
  ReorderedStateNames = sys_quat.StateName;

  % can't use modred, since control surface position (and rate possibley)
  % are part of the system A matrix, so manually re-assemble system
  % matrices for basic plant w/o actuator states
  numActStates = length(act_idx);
  numPlantStates = length(sys_quat.a) - numActStates;
  [numOutputs, numInputs] = size(sys_quat);
  actStateIndices = numPlantStates+1:numPlantStates+numActStates;
  
  Ahat = sys_quat.a(1:numPlantStates,1:numPlantStates);
  Bhat = sys_quat.b(1:numPlantStates,:);
  Bhat(:,10:13) = sys_quat.a(1:numPlantStates, actStateIndices);
  Chat = sys_quat.c(:,1:numPlantStates);
  Dhat = sys_quat.d(:,1:numInputs);
  Dhat(:,10:13) = sys_quat.c(:, actStateIndices);
  sys_quat = ss(Ahat,Bhat,Chat,Dhat);

  sys_quat.StateName = ReorderedStateNames(1:13);
end
%}

% Set input names
sys_quat.InputName = {'LiftEng1';'LiftEng2';'LiftEng3';'LiftEng4';'LiftEng5';'LiftEng6';'LiftEng7';'LiftEng8';'TailEng';
                      'LeftFA';'RightFA';'LeftElev';'RightElev';'Rudder'};

% Replace Bus path names with variable name
sys_quat.OutputName = regexprep(sys_quat.OutputName,'\S*\.','');

% Cleanup
set_param(model       ,'SolverMode','MultiTasking');

% re-enable warnings about missing 'From' for 'Goto'
id = 'Simulink:blocks:MatchingFromNotFound';
warning('on', id);

%% State Transformations
%t          = SimOut.Time;
t          = SimOut.Time.Data;
% VelDtB_bVb = SimOut.AC.AirRel.VelDtB_bWb;
% Asensed_BIb = SimOut.AC.Inertial.Asensed_bIb;
% Vel_bVb    = SimOut.AC.AirRel.Vel_bWb;
Q_h2b      = SimOut.Vehicle.EOM.WorldRelativeData.Q_h2b.Data;
Omeg_BHb   = SimOut.Vehicle.EOM.WorldRelativeData.Omeg_BHb.Data;
QDt_h2b    = 1/2*Qmult(Q_h2b,[zeros(1,numel(t));Omeg_BHb]);
% mu         = SimOut.AC.AirRel.mu;
% gamma      = SimOut.AC.WorldRel.gamma;
% g          = SimIn.Environment.Earth.Gravity.g0(3);

for i = 1:numel(op)
  syst = sys_quat(:,:,i);

  if numel(op) > 1
    %Vel   = interp1(t,Vel_bVb',op(i))';
    %VelDt = interp1(t,VelDtB_bVb',op(i))';
    Q     = interp1(t,Q_h2b',op(i))';
    QDt   = interp1(t,QDt_h2b',op(i))';
  else
    %Vel   = Vel_bVb;
    %VelDt = VelDtB_bVb;
    Q     = Q_h2b;
    QDt   = QDt_h2b;
  end

%{
  % Transform Vel_bEb (u,v,w) to Vtot, alpha, beta
  [DVabDuvw,DuvwDVab,DVabDtDVab] = PDVab(Vel,VelDt);

  States = 'Vel_bEb';
  NewStates = {'Vtot','alpha','beta'};

  StateName = syst.StateName;
  indx = strcmp({States},StateName);
  T  = eye(  numel(indx));
  Ti = eye(  numel(indx));
  P  = zeros(numel(indx));
  T (indx,indx) = DVabDuvw;
  Ti(indx,indx) = DuvwDVab;
  P (indx,indx) = DVabDtDVab;

  syst = stateTrans(syst,T,Ti,P);
  syst.StateName(indx) = NewStates;
%}
  
  % Transform Q_h2b to (~, phi, theta, psi).  For flat Earth Q_i2b = Q_h2b
  [PTERM_Q2E,DEDQ,DQDE] = Q2E_EOM(Q,QDt);
  PTERM_Q2E = blkdiag(0,PTERM_Q2E);
  DEDQ = [zeros(1,4);DEDQ];
  DQDE = [zeros(4,1),DQDE];
  if (SimIn.eomType == EOMEnum.STARS)
    States = 'Q_i2b';
  else
    States = 'EOMStates';
  end
  NewStates = {'~','phi','theta','psi'};

  StateName = syst.StateName;
  indx = strcmp({States},StateName);
  T  = eye(  numel(indx));
  Ti = eye(  numel(indx));
  P  = zeros(numel(indx));
  T (indx,indx) = DEDQ;
  Ti(indx,indx) = DQDE;
  P (indx,indx) = PTERM_Q2E;

  syst = stateTrans(syst,T,Ti,P);
  syst.StateName(indx) = NewStates;

  % Remove empty state "~"
  syst = modred(syst,strcmp({'~'},syst.StateName),'Truncate');

  % Name Outputs
  syst.OutputName = {'Vtot';'alpha';'beta';'vx';'vy';'vz';'ax';'ay';'az';'p';'q';'r';'lat';'lon';'alt';'phi';'theta';'psi'};

  % Create 4th order longitudinal/lateral models
  Xlon=[1 3 5 11]; % States to keep (u,w,q,theta)
  Xlat=[2 4 6 10]; % States to keep (v,p,r,phi)

  % Keep the inputs using indices for the full set of control surface inputs
  Ilon=[9 12:13];  % inputs to keep: TailEng, DelElev
  Ilat=[10:11 14]; % inputs to keep: DelAil, DelRud

  Ylon = [4 6 11 17]; % outputs to keep (u,w,q,theta)
  Ylat = [5 10 12 16]; % outputs to keep (v,p,r,phi)  
  
  numStates = numel(syst.StateName);
  londynt=modred(syst(Ylon,Ilon),setdiff(1:numStates,Xlon),'del');
  latdynt=modred(syst(Ylat,Ilat),setdiff(1:numStates,Xlat),'del');
  
  % Store in array
  sys_euler(:,:,i) = syst;
  londyn_euler(:,:,i) = londynt;
  latdyn_euler(:,:,i) = latdynt;
end

% Rename Quaternion States of Basic Plant
StateNames = sys_quat.StateName;
if (SimIn.eomType == EOMEnum.STARS)
  quat_new_idx  = getIndex(StateNames,'Q_i2b')';
else
  quat_new_idx  = getIndex(StateNames,'EOMStates')';
end
StateNames(quat_new_idx) = {'q0';'q1';'q2';'q3'};
sys_quat.StateName = StateNames;

