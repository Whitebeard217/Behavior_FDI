model = 'GUAM';
% use timeseries input
userStruct.variants.refInputType = 3; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Default(doublets)
time = [0 5:0.5:14]'; % Initial time series with increments of 0.5

% Define trajectory as climbing right-hand turn for 90 deg
% Prescribe inertial position (NED)
pos = [0 0 -1000]; % Inertial Positions (x,y,-z) row vector for each time

% Circular turn parameters
turn_radius = 8000;
%turn_angles = 0:3:90; % Angles for 90° turn (31 points)
turn_angles = 0:1:18; 
%z_descent = (0:30)' * -10 / 10 - 1000; % Climb/descent during turn
z_descent = (0:18)' * -250/25 - 1000;
%z_descent = (0:20)' * -75 - 1000; % Climb/descent during turn

% Generate circular arc
cir_xy = [1500; 0]' + [sind(turn_angles) * turn_radius; ...
                       -cosd(turn_angles) * -turn_radius - turn_radius]';

% Combine circular arc and straight segment into cir_xy
cir_xy = [cir_xy z_descent]; % Append z_descent to cir_xy
% Combine trajectory segments into pos
pos = [pos; cir_xy];

%% Initialize variables based on pos size
N_pos = size(pos, 1); % Use number of rows in pos instead of time
vel = zeros(N_pos, 3);
vel_i = zeros(N_pos, 3);
chi = zeros(N_pos, 1); % Heading is a single column vector
chid = zeros(N_pos, 1); % Heading rate is a single column vector

%% Compute velocity
vel_i(:,1) = gradient(pos(:,1)) ./ gradient(time);
vel_i(:,2) = gradient(pos(:,2)) ./ gradient(time);
vel_i(:,3) = gradient(pos(:,3)) ./ gradient(time);

%% Compute heading
chi = atan2(vel_i(:,2), vel_i(:,1));
chid = gradient(chi) ./ gradient(time);

%% Add stars library blocks for quaternion functions
addpath(genpath('lib'));

%% Compute velocity in heading frame
q = QrotZ(chi);
vel = Qtrans(q, vel_i);

%% Setup trajectory to match bus
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
