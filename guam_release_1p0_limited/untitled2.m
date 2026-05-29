%% sim parameters
model = 'GUAM';
% use timeseries input
userStruct.variants.refInputType = 3; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Default(doublets)
% Define initial time series
time = [0 5:0.5:20]'; % Initial time series with increments of 0.5

% Define trajectory as climbing right-hand turn for 90 deg
% Prescribe inertial position (NED)
pos = [0 0 -500]; % Inertial Positions (x,y,-z) row vector for each time

% Circular turn parameters
turn_radius = 2000;
turn_angles = 0:3:90; % Angles for 90° turn (31 points)
z_descent = (0:30)' * -500 / 25 - 500; % Climb/descent during turn

% Generate circular arc
cir_xy = [1000; 0]' + [sind(turn_angles) * turn_radius; ...
                       cosd(turn_angles) * -turn_radius + turn_radius]';

% Calculate tangent direction parallel to the radius at the end of the arc
end_angle_rad = deg2rad(90); % End angle in radians (90° for a quarter-circle)
radius_direction = [cos(end_angle_rad), sin(end_angle_rad)]; % Radius direction at end angle



% Combine circular arc and straight segment into cir_xy
cir_xy = [cir_xy z_descent]; % Append z_descent to cir_xy
% Define tangent straight segment start and end points
start_pos = cir_xy(end, :); % End of the circular arc
end_pos = start_pos + [1000 * radius_direction(1), 1000 * radius_direction(2), 0]; % Parallel to radius directionnt
% Generate straight segment positions (start and end points only)
straight_segment = [start_pos; end_pos];
cir_xy = [cir_xy; straight_segment]; % Append straight segment to cir_xy

% Combine trajectory segments into pos
pos = [pos; cir_xy];
%% Adjust time series to match pos dimensions
required_time_points = size(pos,1) - length(time); % Calculate additional time points needed
if required_time_points > 0
    additional_time = time(end) + (0.5 * (1:required_time_points))'; % Increment by 0.5 for additional points
    time = [time; additional_time]; % Append additional time points to match pos size
end

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

% figure
% plot3(pos(:,1), pos(:,2), pos(:,3), 'b-o')
% xlabel('X Position')
% ylabel('Y Position')
% zlabel('Altitude (-Z)')
% title('Aircraft Flight Trajectory')
% grid on
% axis equal