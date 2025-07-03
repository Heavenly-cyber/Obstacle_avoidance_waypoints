% 🧭 Simple Waypoints with One Obstacle in MATLAB

clc; close all; clear;

% 1. Define the 3 waypoints (x, y)
waypoints = [
    0,   0;    % WP1
  500, 100;  % WP2
  900, 200   % WP3
  1000, 300
];

% 2. Draw the waypoints and the connecting path
figure; hold on;
plot(waypoints(:,1), waypoints(:,2), 'ro-', 'LineWidth', 2, 'MarkerSize', 8);
xlabel('X (meters)'); ylabel('Y (meters)');
title('Waypoints and Straight Path');
axis equal; grid on;

% 3. Define a circular obstacle
obs_center = [400, 60];  % x=400m, y=60m
obs_radius = 100;          % circle radius = 80m

% 4. Plot the obstacle circle
theta = linspace(0, 2*pi, 100);
xc = obs_center(1) + obs_radius * cos(theta);
yc = obs_center(2) + obs_radius * sin(theta);
plot(xc, yc, 'k--', 'LineWidth', 1.5);

legend('Waypoints & Path', 'Obstacle');

hold off;
