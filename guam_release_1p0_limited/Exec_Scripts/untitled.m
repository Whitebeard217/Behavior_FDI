time = [0 5:0.5:20]'; % Initial time series with increments of 0.5

% Define trajectory as climbing right-hand turn for 90 deg
% Prescribe inertial position (NED)
pos = [0 0 -1000]; % Inertial Positions (x,y,-z) row vector for each time

% Circular turn parameters
turn_radius = 2500;
%turn_angles = 0:3:90; % Angles for 90° turn (31 points)
turn_angles = 0:3:45; 
%z_descent = (0:30)' * -10 / 10 - 1000; % Climb/descent during turn
z_descent = (0:15)' * -10 / 10 - 1000;
%z_descent = (0:20)' * -75 - 1000; % Climb/descent during turn

% Generate circular arc
cir_xy = [1500; 0]' + [sind(turn_angles) * turn_radius; ...
                       cosd(turn_angles) * -turn_radius + turn_radius]';

% Calculate tangent direction parallel to the radius at the end of the arc
%end_angle_rad = deg2rad(90); % End angle in radians (90° for a quarter-circle)
end_angle_rad = deg2rad(45);
radius_direction = [cos(end_angle_rad), sin(end_angle_rad)]; % Radius direction at end angle

% Combine circular arc and straight segment into cir_xy
cir_xy = [cir_xy z_descent]; % Append z_descent to cir_xy
% Define tangent straight segment start and end points
start_pos = cir_xy(end, :); % End of the circular arc
end_pos = start_pos + [14000 * radius_direction(1), 14000 * radius_direction(2), 0]; % Parallel to radius directionnt
% Generate straight segment positions (start and end points only)
straight_segment = end_pos;
cir_xy = [cir_xy; straight_segment]; % Append straight segment to cir_xy

% Combine trajectory segments into pos
pos = [pos; cir_xy];
figure
plot3(pos(:,1), pos(:,2), pos(:,3), 'b-o')
xlabel('X Position')
ylabel('Y Position')
zlabel('Altitude (-Z)')
title('Aircraft Flight Trajectory')
grid on
axis equal
