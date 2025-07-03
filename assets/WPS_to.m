% === SCS UAV Trajectory Simulation with Obstacle-Aware Segment Following ===
clc; clear; close all;

% === Parameters ===
dt = 0.1; T = 1200;
time = 0:dt:T;

% === UAV Parameters ===
max_lat_acc = 6;
L1_default = 40;

% === Waypoints ===
waypoints = [
     0,     0;
   200,    50;
   400,    50;
   400,   250;
   600,   250;
  1000,   150;
  1100,  150;
  1050, -300;
  1000, -500;
   700, -500
];

% === Obstacles ===
obstacles = {
    struct('center', [750, 210], 'radius', 75),
    struct('center', [1070, -100], 'radius', 90)
};
buffer = [20, 40];

% === Static Plot ===
figure; hold on;
plot(waypoints(:,1), waypoints(:,2), 'bo--', 'LineWidth', 1.5);
for i = 1:length(obstacles)
    c = obstacles{i}.center; r = obstacles{i}.radius;
    R = r + buffer(i);
    theta = linspace(0, 2*pi, 100);
    plot(c(1)+r*cos(theta), c(2)+r*sin(theta), 'r--', 'LineWidth', 1.5);
    plot(c(1)+R*cos(theta), c(2)+R*sin(theta), 'r:', 'LineWidth', 1.5);
end
xlabel('X (m)'); ylabel('Y (m)'); axis equal; grid on;
title('Waypoint Path and Obstacles');
legend('Waypoints & Path', 'Obstacle', 'Buffer Zone');

% === Initialization ===
pos = waypoints(1,:); vel = [2.0, 0];
current_seg = 1; mode = 1;
entry_pt = []; exit_pt = [];
angle_traversed = 0; prev_theta = 0;

% === Logging ===
pos_log = []; mode_log = []; ref_log = []; cte_log = []; as_log = [];

% === Simulation Loop ===
for t = time
    if current_seg >= size(waypoints,1)
        break;
    end
    wp_start = waypoints(current_seg,:);
    wp_end = waypoints(current_seg+1,:);
    seg_vec = wp_end - wp_start;
    seg_unit = seg_vec / norm(seg_vec);

    if mode == 1 && isempty(entry_pt)
        for k = 1:length(obstacles)
            c = obstacles{k}.center;
            R = obstacles{k}.radius + buffer(k);
            t_proj = dot(c - wp_start, seg_unit);
            closest_pt = wp_start + t_proj * seg_unit;
            if t_proj >= 0 && t_proj <= norm(seg_vec) && norm(closest_pt - c) < R
                vec_c2start = wp_start - c;
                alpha = acos(R / norm(vec_c2start));
                base_angle = atan2(vec_c2start(2), vec_c2start(1));
                entry_angle = base_angle + alpha - deg2rad(5);
                exit_angle = entry_angle + deg2rad(155);
                entry_pt = c + R * [cos(entry_angle), sin(entry_angle)];
                exit_pt  = c + R * [cos(exit_angle), sin(exit_angle)];
                avoid_center = c; avoid_radius = R;
                mode = 1;
                angle_traversed = 0; prev_theta = atan2(pos(2)-c(2), pos(1)-c(1));
                break;
            end
        end
    end

    switch mode
        case 1
            if isempty(entry_pt)
                target = wp_end;
                if norm(pos - wp_end) < 10
                    current_seg = current_seg + 1;
                    entry_pt = []; exit_pt = [];
                end
            else
                target = entry_pt;
                if norm(pos - entry_pt) < 10
                    mode = 2;
                end
            end

        case 2
            theta = atan2(pos(2)-avoid_center(2), pos(1)-avoid_center(1));
            avoid_vec = [cos(theta), sin(theta)];
            target = avoid_center + avoid_radius * (avoid_vec + 0.3 * [-sin(theta), cos(theta)]);
            dtheta = atan2(sin(theta - prev_theta), cos(theta - prev_theta));
            angle_traversed = angle_traversed + abs(dtheta);
            prev_theta = theta;
            if angle_traversed >= deg2rad(155)
                mode = 3;
            end

        case 3
            target = wp_end;
            if norm(pos - wp_end) < 10
                current_seg = current_seg + 1;
                entry_pt = []; exit_pt = [];
                mode = 1;
            end
    end

    L1_vec = target - pos; V = norm(vel) + 1e-3;
    eta = atan2(vel(1)*L1_vec(2) - vel(2)*L1_vec(1), dot(vel, L1_vec));
    heading_change = acos(dot(seg_unit, vel / norm(vel)));
    if heading_change > deg2rad(60)
        L1 = 25;
    else
        L1 = max(25, 15 + 0.15 * abs(pos(2)));
    end
    cte_weight = min(1, norm(pos - wp_end)/50);
    L1 = max(20, (1 - cte_weight) * 40 + cte_weight * 25);
    a_s = 2 * V^2 / L1 * sin(eta);
    a_s = max(-max_lat_acc, min(max_lat_acc, a_s));
    acc_vec = a_s * [-vel(2), vel(1)] / V;

    vel = vel + acc_vec * dt;
    pos = pos + vel * dt;

    seg_norm = norm(wp_end - wp_start);
    vec_to_pos = pos - wp_start;
    proj_len = dot(vec_to_pos, seg_unit);
    proj_point = wp_start + proj_len * seg_unit;
    if seg_norm > 0
        cte = norm(pos - proj_point);
    else
        cte = 0;
    end

    pos_log(end+1,:) = pos;
    mode_log(end+1) = mode;
    ref_log(end+1,:) = target;
    cte_log(end+1) = cte;
    as_log(end+1) = a_s;
end

% === Final Trajectory Plot ===
figure; hold on;
plot(waypoints(:,1), waypoints(:,2), 'bo--', 'LineWidth', 1.5);
plot(pos_log(:,1), pos_log(:,2), 'k-', 'LineWidth', 2);
for k = 1:length(obstacles)
    c = obstacles{k}.center; r = obstacles{k}.radius;
    R = r + buffer(k); theta = linspace(0, 2*pi, 100);
    plot(c(1)+r*cos(theta), c(2)+r*sin(theta), 'r--', 'LineWidth', 1.5);
    plot(c(1)+R*cos(theta), c(2)+R*sin(theta), 'r:', 'LineWidth', 1.5);
end
xlabel('X (m)'); ylabel('Y (m)'); axis equal; grid on;
title('UAV Trajectory with SCS Obstacle Avoidance');
legend('Waypoints', 'UAV Path', 'Obstacle', 'Buffer Zone');

% === Cross-track Error ===
figure;
plot(time(1:length(cte_log)), cte_log, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Cross-Track Error (m)');
title('Cross-Track Error Over Time'); grid on;

% === Mode Transitions ===
figure;
plot(time(1:length(mode_log)), mode_log, 'm', 'LineWidth', 1.5);
ylabel('Mode'); xlabel('Time (s)');
title('Mode Transitions Over Time'); grid on;

% === Compute Heading/Curvature & Smooth ===
vel_log = diff(pos_log) / dt; vel_log = [vel_log; vel_log(end,:)];
V_log = sqrt(sum(vel_log.^2, 2));
heading = unwrap(atan2(vel_log(:,2), vel_log(:,1)));
heading_rate = [diff(heading)/dt; 0];
curvature = heading_rate ./ (V_log + 1e-3);
curvature_derivative = [diff(curvature)/dt; 0];

% === Smooth using Curve Fitting Toolbox and Signal Processing ===
try
    % Curve fitting toolbox (spline-based)
    sp_kappa = csaps(time(1:length(curvature)), curvature, 1.0000);
    curvature_csaps = fnval(sp_kappa, time(1:length(curvature)));
    
    sp_dkappa = csaps(time(1:length(curvature_derivative)), curvature_derivative, 1.0000);
    curvature_derivative_csaps = fnval(sp_dkappa, time(1:length(curvature_derivative)));

    sp_as = csaps(time(1:length(as_log)), as_log, 1.0000);
    as_log_csaps = fnval(sp_as, time(1:length(as_log)));

    % Further smoothing using Savitzky-Golay filter (Signal Processing Toolbox)
    window_size = 51; % increase window size for stronger smoothing
    poly_order = 2;  % lower polynomial order to reduce oscillations
    curvature = sgolayfilt(curvature_csaps, poly_order, window_size);
    curvature_derivative = sgolayfilt(curvature_derivative_csaps, poly_order, window_size);
    as_log = sgolayfilt(as_log_csaps, poly_order, window_size);

    % Final layer: Moving average to flatten residuals
    curvature = movmean(curvature, 25);
    curvature_derivative = movmean(curvature_derivative, 25);
    as_log = movmean(as_log, 25);

    warning('Curve Fitting Toolbox required for smoothing. Using raw values.');
end

% === Plot Smoothed Curvature ===
figure;
plot(time(1:length(curvature)), curvature, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Curvature (1/m)');
title('Smoothed UAV Path Curvature Over Time'); grid on;

figure;
plot(time(1:length(curvature_derivative)), curvature_derivative, 'k', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('d(Curvature)/dt (1/m^2)');
title('Smoothed Derivative of Curvature Over Time'); grid on;

figure;
plot(time(1:length(as_log)), as_log, 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Lateral Acceleration a_s (m/s^2)');
title('Smoothed Lateral Acceleration Over Time'); grid on;
