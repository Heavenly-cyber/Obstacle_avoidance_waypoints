clc; clear; close all;

% 1. Finalized Waypoints (as per your message)
waypoints = [
     0,     0;       % WP1
   200,    50;       % WP2
   400,    50;       % WP3
   400,   250;       % WP4
   600,   250;       % WP5
   900,   150;       % WP6
  1100,  150;        % WP7
  1050, -300;        % WP8
  1000, -500;        % WP9
   700, -500         % WP10
];

% 2. Plotting the Path
figure; hold on;
plot(waypoints(:,1), waypoints(:,2), 'bo-', ...
    'LineWidth', 2, 'MarkerSize', 8);
xlabel('X (m)'); ylabel('Y (m)');
title('Finalized Waypoint Path with Two Obstacles');
axis equal; grid on;

% 3. Obstacles
obs1_center = [750, 210];
obs1_radius = 90;

obs2_center = [1070, -100];
obs2_radius = 90;

% Buffer distance (safety margin)
buffer_dist = 40;

% 4. Plot Original Obstacles
theta = linspace(0, 2*pi, 100);
x1 = obs1_center(1) + obs1_radius * cos(theta);
y1 = obs1_center(2) + obs1_radius * sin(theta);
plot(x1, y1, 'r--', 'LineWidth', 1.5);

x2 = obs2_center(1) + obs2_radius * cos(theta);
y2 = obs2_center(2) + obs2_radius * sin(theta);
plot(x2, y2, 'r--', 'LineWidth', 1.5);

% 5. Plot Buffer Zones (Outer Circles)
x1_buffer = obs1_center(1) + (obs1_radius + buffer_dist) * cos(theta);
y1_buffer = obs1_center(2) + (obs1_radius + buffer_dist) * sin(theta);
plot(x1_buffer, y1_buffer, 'r:', 'LineWidth', 1.5);

x2_buffer = obs2_center(1) + (obs2_radius + buffer_dist) * cos(theta);
y2_buffer = obs2_center(2) + (obs2_radius + buffer_dist) * sin(theta);
plot(x2_buffer, y2_buffer, 'r:', 'LineWidth', 1.5);

% 6. Legend
legend('Waypoints & Path', 'Obstacle 1', 'Obstacle 2', ...
       'Buffer Zone 1', 'Buffer Zone 2');

hold off;
