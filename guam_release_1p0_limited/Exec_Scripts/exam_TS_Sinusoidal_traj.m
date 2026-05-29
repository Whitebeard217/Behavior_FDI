%% sim parameters
model = 'GUAM';
% use timeseries input
userStruct.variants.refInputType=3; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Default(doublets)

%% setup trajectory and pass to target
time = (0:1:20)';
N_time = length(time);

vel     = zeros(3,N_time)';
vel_i   = zeros(3,N_time)';
pos     = zeros(3,N_time)';
chi     = zeros(1,N_time)';
chid    = zeros(1,N_time)';

% define trajectory as sinusoidal variations
% prescribe position (NED)
% pos(:,1) = time;
% pos(:,2) = 10*cos(2*pi/300*time);
% pos(:,3) = 10*sin(2*pi/300*time);
time = (0:1:20)'; % Time vector from 0 to 20 seconds
pos(:,1) = time; % X-axis: Linear progression (straight line)
pos(:,2) = time; % Y-axis: Linear progression (straight line)
pos(:,3) = -500; % Z-axis: Constant offset of 500

% compute velocity
vel_i(:,1) = gradient(pos(:,1))./gradient(time);
vel_i(:,2) = gradient(pos(:,2))./gradient(time);
vel_i(:,3) = gradient(pos(:,3))./gradient(time);

% compute heading
chi     = atan2(vel_i(:,2),vel_i(:,1));
chid    = gradient(chi)./gradient(time);

% add stars library blocks for quaternion functions
addpath(genpath('lib'));

% compute velocity in heading frame
q = QrotZ(chi);
vel = Qtrans(q,vel_i);

% setup trajectory to match bus
RefInput.Vel_bIc_des    = timeseries(vel,time);
RefInput.pos_des        = timeseries(pos,time);
RefInput.chi_des        = timeseries(chi,time);
RefInput.chi_dot_des    = timeseries(chid,time);
RefInput.vel_des        = timeseries(vel_i,time);

target.RefInput = RefInput;

%% Prepare to run simulation
% set initial conditions and add trajectory to SimInput
simSetup;
open(model);