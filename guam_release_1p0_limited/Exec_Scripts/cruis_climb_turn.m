%% sim parameters
model = 'GUAM';
% use timeseries input
userStruct.variants.refInputType=3; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Default(doublets)

%% setup trajectory and pass to target
%time        = [0 10:1:40]'; % Column vector of time points
time = [0 5:0.5:15]';
N_time = length(time);

vel     = zeros(N_time, 3);
vel_i   = zeros(N_time, 3);
pos     = zeros(N_time, 3);
chi     = zeros(N_time, 3);
chid    = zeros(N_time, 3);

% define trajectory as climbing right hand turn for 90 deg
% prescibe inertial position (NED)
pos     = [0 0 -1500]; % Inertial Positions (x,y,-z) row vector for each time
%cir_xy = [2000;0]'+[sind([0:3:90])*4000; cosd([0:3:90])*-4000+4000]';
cir_xy = [1500;0]' + [sind([0:1:20])*6000; -cosd([0:1:20])*-6000 - 6000]';

pos = [pos;[cir_xy (0:20)'*-250/25-1500]];
% Compute velocity
vel_i(:,1)  = gradient(pos(:,1))./gradient(time); 
vel_i(:,2)  = gradient(pos(:,2))./gradient(time); 
vel_i(:,3)  = gradient(pos(:,3))./gradient(time); 

% Compute heading
chi     = atan2(vel_i(:,2),vel_i(:,1));
chid    = gradient(chi)./gradient(time);

% add stars library blocks for quaternion functions
addpath(genpath('lib'));

% compute velocity in heading frame
q = QrotZ(chi);
vel = Qtrans(q,vel_i);

% setup trajectory to match bus
RefInput.Vel_bIc_des    = timeseries(vel,time); % Heading frame velocity
RefInput.pos_des        = timeseries(pos,time); % Inertial Position
RefInput.chi_des        = timeseries(chi,time); % Heading Angle
RefInput.chi_dot_des    = timeseries(chid,time); % Heading Angle Rate
RefInput.vel_des        = timeseries(vel_i,time); % Inertial Position

target.RefInput = RefInput;

%% Prepare to run simulation
% set initial conditions and add trajectory to SimInput
simSetup;
open(model);