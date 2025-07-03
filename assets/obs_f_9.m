clc; clear; close all;

% Parameters
dt = 0.1; T = 400;
time = 0:dt:T;
N = length(time);

% UAV parameters
max_lat_acc = 10;
target_line_y = 0;
circle_center = [0, 0];
inner_radius = 60;
outer_radius = 100;
exit_angle_deg = 155;

% Transition durations
transition_time = 5;             % for exit smoothing
transition_steps = round(transition_time / dt);
transition_in_time = 5;          % for entry smoothing
transition_in_steps = round(transition_in_time / dt);

% Initialization
pos = [-400, 50];
vel = [2.5, 0];
mode = 1;
entry_point = []; exit_point = [];
entry_logged = false; exit_logged = false;
transition_counter = 0;
transition_in_counter = 0;

% Logs
pos_log = zeros(N, 2);
ref_log = zeros(N, 2);
mode_log = zeros(N, 1);
curvature_log = zeros(N, 1);

for i = 1:N
    pos_log(i,:) = pos;
    mode_log(i) = mode;

    % Obstacle distances
    d_to_center = norm(pos - circle_center);

    % --- MODE SWITCHING LOGIC ---
    entry_trigger_radius = outer_radius + 30;
    if mode == 1 && d_to_center < entry_trigger_radius
        mode = 2;
        theta_entry = atan2(pos(2)-circle_center(2), pos(1)-circle_center(1));
        entry_point = circle_center + outer_radius * [cos(theta_entry), sin(theta_entry)];
        entry_logged = true;
        transition_in_counter = transition_in_steps;
    elseif mode == 2 && ~exit_logged
        theta_exit = theta_entry + deg2rad(exit_angle_deg);
        exit_point = circle_center + outer_radius * [cos(theta_exit), sin(theta_exit)];
        if norm(pos - exit_point) < 5
            mode = 3;
            exit_logged = true;
            transition_counter = transition_steps;
            exit_heading = [-sin(theta_exit), cos(theta_exit)];
        end
    end

    % --- REFERENCE POINT SELECTION ---
    if mode == 1 || (mode == 3 && transition_counter == 0)
        ref_point = pos + 60 * [1, 0];
        ref_point(2) = target_line_y;

    elseif mode == 2
        theta = atan2(pos(2)-circle_center(2), pos(1)-circle_center(1));
        tangent_vec = [-sin(theta), cos(theta)];
        
        if transition_in_counter > 0
            blend_in = 1 - transition_in_counter / transition_in_steps;
            blended_dir = (1 - blend_in) * [1, 0] + blend_in * tangent_vec;
            blended_dir = blended_dir / norm(blended_dir);
            ref_point = pos + 60 * blended_dir;
            transition_in_counter = transition_in_counter - 1;
        else
            ref_point = circle_center + outer_radius * [cos(theta + 0.3), sin(theta + 0.3)];
        end

    elseif mode == 3 && transition_counter > 0
        blend = 1 - transition_counter / transition_steps;
        blended_dir = (1 - blend) * exit_heading + blend * [1, 0];
        blended_dir = blended_dir / norm(blended_dir);
        ref_point = pos + 60 * blended_dir;
        ref_point(2) = target_line_y;
        transition_counter = transition_counter - 1;
    end
    ref_log(i,:) = ref_point;

    % --- GUIDANCE LAW ---
    L1_vec = ref_point - pos;
    V = norm(vel) + 1e-6;
    eta = atan2(vel(1)*L1_vec(2) - vel(2)*L1_vec(1), dot(vel, L1_vec));
    L1 = max(40, 20 + 0.2 * abs(pos(2)));
    a_s = 2 * V^2 / L1 * sin(eta);
    a_s = max(-max_lat_acc, min(max_lat_acc, a_s));
    acc_vec = a_s * [-vel(2), vel(1)] / V;

    vel = vel + acc_vec * dt;
    pos = pos + vel * dt;

    % --- Log curvature ---
    curvature_log(i) = abs(eta) / norm(L1_vec);
end

% --- PLOT RESULTS ---
figure;
plot(pos_log(:,1), pos_log(:,2), 'b', 'LineWidth', 2); hold on;
viscircles(circle_center, inner_radius, 'LineStyle', '--', 'Color', 'r');
viscircles(circle_center, outer_radius, 'LineStyle', '--', 'Color', 'k');
yline(target_line_y, '--g', 'Target Line', 'LabelHorizontalAlignment','left');
if entry_logged, plot(entry_point(1), entry_point(2), 'mo', 'MarkerSize', 10, 'LineWidth', 2); end
if exit_logged, plot(exit_point(1), exit_point(2), 'co', 'MarkerSize', 10, 'LineWidth', 2); end
legend('UAV Path', 'Obstacle', 'Danger Circle', 'Target Line', 'Entry', 'Exit');
title('SCS Guidance with Smooth Transitions (Blended Entry & Exit)');
xlabel('X [m]'); ylabel('Y [m]'); axis equal; grid on;

% --- CURVATURE PLOT ---
figure;
plot(time, curvature_log, 'r', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Curvature [1/m]');
title('UAV Path Curvature Over Time'); grid on;
