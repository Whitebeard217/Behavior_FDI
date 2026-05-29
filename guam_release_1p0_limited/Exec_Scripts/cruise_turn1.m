%% sim parameters
model = 'GUAM';
% use timeseries input
userStruct.variants.refInputType=3; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Default(doublets)

%% setup trajectory and pass to target
%time        = [0 10:1:40]'; % Column vector of time points
time = [0 5:1:15]';

% define trajectory as climbing right hand turn for 90 deg
% prescibe inertial position (NED)
pos     = [0 0 -1000]; % Inertial Positions (x,y,-z) row vector for each time
%cir_xy = [2000;0]'+[sind([0:3:90])*4000; cosd([0:3:90])*-4000+4000]'; %original
%cir_xy = [1500;0]' + [sind([0:1:10])*8000; -cosd([0:1:10])*-8000 - 8000]';
cir_xy = [1500;0]' + [sind([0:2:20])*6000; -cosd([0:2:20])*-6000 - 6000]';
%end_angle_rad = deg2rad(10);
end_angle_rad = deg2rad(45);
radius_direction = [cos(end_angle_rad), sin(end_angle_rad)]; % Radius direction at end angle
%z_descend = (0:10)'*-10/10-1000;
z_descend = (0:10)'*-10/10-1000;
cir_xy = [cir_xy z_descend];
%pos = [pos;[cir_xy (0:20)'*-500/25-1000]];
% Define tangent straight segment start and end points
start_pos = cir_xy(end, :); % End of the circular arc
end_pos = start_pos + [1500 * radius_direction(1), -1500 * radius_direction(2), 0]; % Parallel to radius directionnt
% Generate straight segment positions (start and end points only)
straight_segment = end_pos;

cir_xy = [cir_xy; straight_segment]; % Append straight segment to cir_xy
pos = [pos; cir_xy];
required_time_points = size(pos,1) - length(time); % Calculate additional time points needed
if required_time_points > 0
    additional_time = time(end) + (0.5 * (1:required_time_points))'; % Increment by 0.5 for additional points
    time = [time; additional_time]; % Append additional time points to match pos size
end
time(length(time),1) = time(length(time)-1,1)+5;

%% Initialize variables based on pos size
N_pos = size(pos, 1); % Use number of rows in pos instead of time
vel = zeros(N_pos, 3);
vel_i = zeros(N_pos, 3);
chi = zeros(N_pos, 1); % Heading is a single column vector
chid = zeros(N_pos, 1); % Heading rate is a single column vector
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