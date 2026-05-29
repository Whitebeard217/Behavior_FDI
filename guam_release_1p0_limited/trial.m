% Parameters
arc_radius = 500;       % Radius of the main semicircular arc
straight_length = 500;  % Length of straight segments on each side
transition_radius = 200; % Radius for transition arcs
points_per_segment = 100; % Points for smooth plotting

% 1. First straight segment (unchanged)
x1 = linspace(-straight_length-arc_radius, -arc_radius-transition_radius, points_per_segment)+1000;
y1 = zeros(size(x1));

% 2. First transition arc (mirrored horizontally, shifted up by 200)
theta1 = linspace(0, pi/2, points_per_segment);
x2 = -(arc_radius - transition_radius * cos(theta1))-200+1000;   % MIRRORED horizontally (x)
y2 = -transition_radius * sin(theta1) + 200;            % shifted up

% 3. Main semicircular arc (shifted up by 200)
theta2 = linspace(pi, 0, points_per_segment);
x3 = arc_radius * cos(theta2)+1000;
y3 = arc_radius * sin(theta2) + 200;

% 4. Second transition arc (mirrored horizontally, shifted up by 200)
theta3 = linspace(pi/2, 0, points_per_segment);
x4 = arc_radius + transition_radius * -cos(theta3)+200+1000;      % MIRRORED horizontally (x)
y4 = -transition_radius * sin(theta3) + 200;            % shifted up

% 5. Second straight segment (unchanged)
x5 = linspace(arc_radius+transition_radius, arc_radius+straight_length, points_per_segment)+1000;
y5 = zeros(size(x5));

% Plot segments
figure; hold on;
plot(x1, y1, 'k-', 'LineWidth', 2);           % First straight
plot(x2, y2, 'b-', 'LineWidth', 2);           % First blue arc (mirrored, shifted)
plot(x3, y3, 'r-', 'LineWidth', 2);           % Red arc (shifted)
plot(x4, y4, 'b-', 'LineWidth', 2);           % Second blue arc (mirrored, shifted)
plot(x5, y5, 'k-', 'LineWidth', 2);           % Second straight

xlabel('X Position');
ylabel('Y Position');
title('Aircraft Trajectory');
grid on;
axis equal;