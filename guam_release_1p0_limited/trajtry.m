% Parameters
arc_radius = 8000;       % Radius of the main semicircular arc
straight_length = 1200;  % Length of straight segments on each side
transition_radius = 6000; % Radius for transition arcs
points_per_segment = 15; % Points for smooth plotting

% 1. First straight segment (shifted in x)
x1_start = -straight_length-arc_radius + 1000 + 1200 + 11800;
x1_end = -arc_radius-transition_radius + 1000 + 1200 + 11800;
x1 = linspace(x1_start, x1_end, 2)';
y1 = zeros(2, 1);

% 2. First transition arc (mirrored horizontally, shifted)
theta1 = linspace(pi/6, pi/2, points_per_segment)';
x2 = -(arc_radius - transition_radius * cos(theta1))-200+1000+1200+10800;
y2 = -transition_radius * sin(theta1) + 200+5800;

% 3. Main semicircular arc (shifted and adjusted)
theta2 = linspace(2*pi/3, pi/3, points_per_segment)';
x3 = arc_radius * cos(theta2)+ 1000 - 26.7949 + 700 + 2326.8 + 9996.15;
y3 = arc_radius * sin(theta2) + 200 - 800 - 166.025 - 3162.18;

% 4. Second transition arc (mirrored horizontally, shifted)
theta3 = linspace(pi/2, pi/6, points_per_segment)';
x4 = arc_radius + transition_radius * -cos(theta3) + 200 - 53.5898 + 500 + 700 + 13850;      % MIRRORED horizontally (x)
y4 = -transition_radius * sin(theta3) + 200 + 5800;     

% 5. Second straight segment (shifted)
x5_start = arc_radius + transition_radius - 53.5898 + 500 + 700 + 12850;
x5_end = arc_radius + straight_length - 53.5898 + 500 + 700 + 12850;
x5 = linspace(x5_start, x5_end, 2)';
y5 = zeros(2, 1);

% Concatenate all segments
pos_x = [x1; x2; x3; x4; x5];
pos_y = [y1; y2; y3; y4; y5];

% Assuming constant altitude (z)
altitude = -1000; % NED altitude (negative down)
pos_z = altitude * ones(size(pos_x));

% Combine into position matrix [x, y, z]
pos = [pos_x, pos_y, pos_z];



figure
plot3(pos(:,1), pos(:,2), pos(:,3), 'b-o')
xlabel('X Position')
ylabel('Y Position')
zlabel('Altitude (-Z)')
title('Aircraft Flight Trajectory')
grid on
hold on
axis equal