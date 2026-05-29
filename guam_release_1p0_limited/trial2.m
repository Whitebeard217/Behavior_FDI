% Parameters
arc_radius = 8000;       % Radius of the main semicircular arc
straight_length = 1200;  % Length of straight segments on each side
transition_radius = 6000; % Radius for transition arcs
points_per_segment = 20; % Points for smooth plotting

% 1. First straight segment (unchanged)
x1 = linspace(-straight_length-arc_radius, -arc_radius-transition_radius, 2)+1000+1200+11800;
y1 = zeros(size(x1));

% 2. First transition arc (mirrored horizontally, shifted up by 200)
theta1 = linspace(pi/6, pi/2, points_per_segment);
x2 = -(arc_radius - transition_radius * cos(theta1))-200+1000+1200+10800;   % MIRRORED horizontally (x)
y2 = -transition_radius * sin(theta1) + 200+5800;            % shifted up

% 3. Main semicircular arc (shifted up by 200)
theta2 = linspace(2*pi/3, pi/3, points_per_segment);
x3 = arc_radius * cos(theta2)+1000-26.7949+700+2326.8+9996.15;
y3 = arc_radius * sin(theta2) + 200 - 800 - 166.025-3162.18;

% 4. Second transition arc (mirrored horizontally, shifted up by 200)
theta3 = linspace(pi/2, pi/6, points_per_segment);
x4 = arc_radius + transition_radius * -cos(theta3)+200-53.5898+500+700+13850;      % MIRRORED horizontally (x)
y4 = -transition_radius * sin(theta3) + 200+5800;            % shifted up

% 5. Second straight segment (unchanged)
x5 = linspace(arc_radius+transition_radius, arc_radius+straight_length, 2)-53.5898+500+700+12850;
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