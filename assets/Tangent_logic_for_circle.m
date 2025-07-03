clc; clear; close all;

% UAV and Circle Parameters
P = [-400, 50];        % UAV Position
C = [0, 0];            % Circle Center
R = 100;               % Outer circle radius

% Vector from circle to UAV
V = P - C;
d = norm(V);

if d <= R
    error('No tangent possible, UAV is inside or on the circle.');
end

% Unit vector from circle center to UAV
u = V / d;

% Perpendicular vector in 2D (rotate 90°)
perp = [-u(2), u(1)];

% Distance from center to tangent point along circle
h = sqrt(d^2 - R^2);
tangent_dist = R / d;

% Tangent points
T1 = C + R * (tangent_dist * u + sqrt(1 - tangent_dist^2) * perp);
T2 = C + R * (tangent_dist * u - sqrt(1 - tangent_dist^2) * perp);

% Plot the circle
theta = linspace(0, 2*pi, 300);
circle_x = C(1) + R * cos(theta);
circle_y = C(2) + R * sin(theta);

% Plotting
figure;
plot(circle_x, circle_y, 'k--', 'LineWidth', 2); hold on;
plot([P(1), T1(1)], [P(2), T1(2)], 'b', 'LineWidth', 2);
plot([P(1), T2(1)], [P(2), T2(2)], 'g', 'LineWidth', 2);
plot(P(1), P(2), 'kx', 'MarkerSize', 10, 'LineWidth', 1.5);
plot(T1(1), T1(2), 'mo', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(T2(1), T2(2), 'co', 'MarkerSize', 8, 'LineWidth', 1.5);

axis equal;
xlabel('X (m)');
ylabel('Y (m)');
title('Correct Tangents from UAV to Outer Circle (Danger Zone)');
legend('Obstacle (Outer Circle)', 'Tangent Line 1', 'Tangent Line 2', ...
       'UAV Start', 'Tangent Point 1', 'Tangent Point 2');
grid on;
