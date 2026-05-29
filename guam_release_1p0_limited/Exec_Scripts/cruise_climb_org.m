%% sim parameters
model = 'GUAM';
userStruct.variants.refInputType = 3; % Timeseries

%% setup trajectory and pass to target
time = [0; 5; 8; 11]; % Same time difference (first two points 0s, 5s)
N_time = length(time);
vel_i   = zeros(N_time, 3);
% Straight segments after initial point, each 1km forward, climb 250m up (z negative in NED)
step_length = 1000; % meters forward per segment
step_climb = -250/5; % per second, so per segment -50

pos = zeros(N_time,3);
pos(1,:) = [0 0 -1000];

for i = 2:N_time
    pos(i, 1) = pos(i-1, 1) + step_length;         % advance straight in x
    pos(i, 2) = 0;                          % stay straight (y constant)
    pos(i, 3) = pos(i-1, 3) + step_climb;         % climb
end

% Compute velocity
vel_i(:,1) = gradient(pos(:,1))./gradient(time); 
vel_i(:,2) = gradient(pos(:,2))./gradient(time); 
vel_i(:,3) = gradient(pos(:,3))./gradient(time); 

% Compute heading (should be near zero, straight ahead)
chi = atan2(vel_i(:,2), vel_i(:,1));
chid = gradient(chi)./gradient(time);

addpath(genpath('lib'));
q = QrotZ(chi);
vel = Qtrans(q, vel_i);

RefInput.Vel_bIc_des = timeseries(vel, time);
RefInput.pos_des     = timeseries(pos, time);
RefInput.chi_des     = timeseries(chi, time);
RefInput.chi_dot_des = timeseries(chid, time);
RefInput.vel_des     = timeseries(vel_i, time);

target.RefInput = RefInput;

%% Prepare to run simulation
simSetup;
open(model);