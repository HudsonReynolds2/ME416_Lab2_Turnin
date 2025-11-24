clear; clc; close all;

%% ======================= AUTONOMOUS MAZE NAVIGATION =======================
% This script implements autonomous robot navigation through a maze using:
%   1. Extended Kalman Filter (EKF) for state estimation
%   2. Dubins path planning for smooth, kinematically-feasible trajectories
%   3. Lateral error feedback control for path following
%
% The EKF fuses motion commands (prediction) with motion capture measurements
% (correction) to estimate the robot's state: [x, y, theta, v, omega]
%
% Uses HARD-CODED waypoints (from limo_config) with Dubins smoothing
% Transform 4 with positive yaw (verified correct)

fprintf('=================================================================\n');
fprintf('AUTONOMOUS MAZE NAVIGATION (HARDCODED WAYPOINTS)\n');
fprintf('=================================================================\n\n');

% Load configuration
cfg = limo_config();

%% ======================= SELECT MAZE =======================
fprintf('Available mazes:\n');
for i = 1:length(cfg.mazes)
    fprintf('  %d. %s\n', i, cfg.mazes(i).name);
end
maze_choice = input(sprintf('Select maze (1-%d): ', length(cfg.mazes)));

if maze_choice < 1 || maze_choice > length(cfg.mazes)
    error('Invalid maze selection!');
end

% Set selected maze (MATCHES VISUALIZER BEHAVIOR)
cfg.maze_select    = maze_choice;
cfg.obs            = cfg.mazes(maze_choice).obs;
cfg.start_pos      = cfg.mazes(maze_choice).start_pos;
cfg.goal_pos       = cfg.mazes(maze_choice).goal_pos;
cfg.arena_bounds   = cfg.mazes(maze_choice).arena_bounds;
cfg.courseName     = cfg.mazes(maze_choice).name;
cfg.waypoints      = cfg.mazes(maze_choice).waypoints;   % <-- IMPORTANT

fprintf('\n✓ Selected: %s\n', cfg.courseName);

%% ======================= LOAD CALIBRATION =======================
if ~exist(cfg.CALIB_FILE, 'file')
    error(['Calibration file not found!\n' ...
           'Run calibrate_limo.m first with robot at origin']);
end
load(cfg.CALIB_FILE, 'calib');
fprintf('✓ Calibration loaded (Transform %d in file)\n', calib.transform);

% Force Transform 4 (verified correct)
calib.transform = 4;
fprintf('✓ Using Transform 4 with positive yaw (verified)\n');

%% ======================= BUILD PATH FROM HARDCODED WAYPOINTS =======================
fprintf('\n=== PATH PLANNING (HARDCODED WAYPOINTS) ===\n');

waypoints = cfg.waypoints;   % exactly the same as visualizer

fprintf('Using %d hardcoded waypoints from limo_config()...\n', size(waypoints,1));
disp(waypoints);

fprintf('Building Dubins path through hardcoded waypoints...\n');
try
    [ref, ~] = build_dubins_path(waypoints, cfg.R_min, cfg.step, ...
                                 cfg.v_nom, cfg.obs, cfg.obsR);
catch ME
    error('Dubins path generation FAILED: %s', ME.message);
end

fprintf('✓ Path complete: %d points, %.2f m\n', length(ref.x), ref.s(end));

%% ======================= SETUP CONNECTIONS =======================
fprintf('\n=== CONNECTING ===\n');

% MQTT for pose data
mq = mqttclient(cfg.broker, Port=cfg.mqtt_port, ClientID="autoNav");
if ~mq.Connected
    error('Could not connect to MQTT broker');
end
subscribe(mq, cfg.topic_pose);
fprintf('✓ MQTT connected\n');

% TCP socket for robot control
sock = tcpclient(cfg.LIMO_IP, cfg.LIMO_PORT);
configureCallback(sock, "off");
fprintf('✓ TCP connected to robot\n');

%% ======================= INITIALIZE EKF =======================
% Extended Kalman Filter (EKF) Setup
% The EKF maintains a 5-dimensional state vector:
%   x_ekf = [x, y, theta, v, omega]'
% where:
%   x, y    - Position in maze frame [m]
%   theta   - Heading angle [rad]
%   v       - Linear velocity [m/s]
%   omega   - Angular velocity [rad/s]
%
% The covariance matrix P tracks uncertainty in each state variable.
% Initial covariance P0 reflects our initial uncertainty about the state.

fprintf('\n=== INITIALIZING EKF ===\n');

% Get initial pose from motion capture system
pose = get_mocap_pose(mq, cfg.topic_pose);
[x_init, y_init, theta_init] = transform_pose(pose, calib);

% Initialize state vector with measured pose and zero velocities
x_ekf = [x_init; y_init; theta_init; 0; 0];
% Initialize covariance matrix (uncertainty in initial state estimate)
P_ekf = cfg.P0;

fprintf('✓ Initial pose: (%.2f, %.2f) @ %.0f°\n', ...
        x_init, y_init, rad2deg(theta_init));

% Check if robot is near start of this exact path
start_error = hypot(x_init - ref.x(1), y_init - ref.y(1));
if start_error > 0.3
    fprintf('\n⚠ WARNING: Robot is %.2f m from path start!\n', start_error);
    fprintf('  Expected: (%.2f, %.2f)\n', ref.x(1), ref.y(1));
    fprintf('  Actual:   (%.2f, %.2f)\n', x_init, y_init);
    response = input('Continue anyway? (y/n): ', 's');
    if ~strcmpi(response, 'y')
        stopRobot(sock, cfg.WHEELBASE);
        clear sock;
        delete(mq);
        return;
    end
end

%% ======================= SETUP VISUALIZATION =======================
fprintf('\nSetting up visualization...\n');
setup_figure_fullscreen();

% Main plot
subplot(1,2,1);
hold on; grid on; axis equal;
xlim([cfg.arena_bounds(1) cfg.arena_bounds(2)]);
ylim([cfg.arena_bounds(3) cfg.arena_bounds(4)]);
title(sprintf('%s - Autonomous Navigation', cfg.courseName), ...
      'FontSize', 16, 'FontWeight', 'bold');
xlabel('X [m]'); ylabel('Y [m]');

% Static elements
plot_arena(cfg.arena_bounds);
plot_obstacles(cfg.obs, cfg.obsR);
plot_path(ref, waypoints);

% Dynamic elements
h_trail = plot(NaN, NaN, 'b-', 'LineWidth', 3);
h_robot = plot(NaN, NaN, 'bo', 'MarkerSize', 20, 'MarkerFaceColor', 'b');
h_arrow = quiver(0, 0, 0, 0, 'b', 'LineWidth', 4, 'MaxHeadSize', 0.5);
h_target = plot(NaN, NaN, 'rs', 'MarkerSize', 20, 'LineWidth', 3);

legend({'Arena', 'Obstacles', '', 'Waypoints', 'Path', 'Start', ...
        'Trail', 'Robot', 'Target'}, 'Location', 'northwest');

% Info panel
subplot(1,2,2);
axis off;
h_info = text(0.05, 0.95, '', 'FontSize', 11, 'FontName', 'Courier', ...
              'VerticalAlignment', 'top', 'Interpreter', 'none');

fprintf('✓ Visualization ready\n');

%% ======================= MAIN CONTROL LOOP =======================
fprintf('\n=== STARTING AUTONOMOUS NAVIGATION ===\n');
fprintf('Press Ctrl+C to stop\n\n');

% Initialize logging
log = struct();
log.time = [];
log.x_meas = []; log.y_meas = []; log.theta_meas = [];
log.x_ekf = []; log.y_ekf = []; log.theta_ekf = [];
log.v_ekf = []; log.omega_ekf = [];
log.x_ref = []; log.y_ref = []; log.theta_ref = [];
log.e_y = []; log.e_theta = [];
log.v_cmd = []; log.omega_cmd = [];
log.s_current = [];

trail_x = [];
trail_y = [];

t_start = tic;
iter = 0;
goal_reached = false;
v_cmd = 0;
omega_cmd = 0;

try
    while ~goal_reached && toc(t_start) < cfg.TIMEOUT
        iter = iter + 1;
        t = toc(t_start);
        
        %% --- EKF PREDICTION ---
        % Predict step: Propagate state forward using motion model
        % Uses unicycle kinematics with first-order velocity dynamics
        % x_k+1 = f(x_k, u_k) + process noise
        % Covariance grows due to process noise Q
        [x_ekf, P_ekf] = ekf_predict(x_ekf, P_ekf, v_cmd, omega_cmd, ...
                                      cfg.dt_control, cfg.Q, cfg.TAU_V, cfg.TAU_OMEGA);

        %% --- GET MEASUREMENT ---
        % Read position and orientation from motion capture system
        pose = get_mocap_pose(mq, cfg.topic_pose);
        [x_meas, y_meas, theta_meas] = transform_pose(pose, calib);

        %% --- EKF UPDATE ---
        % Correct step: Fuse prediction with measurement using Kalman gain
        % K = P * H' * inv(H * P * H' + R)
        % The gain K balances trust between prediction and measurement
        % based on their respective uncertainties (P and R)
        z = [x_meas; y_meas; theta_meas];
        [x_ekf, P_ekf] = ekf_update(x_ekf, P_ekf, z, cfg.R);
        
        %% --- EXTRACT ESTIMATES ---
        x_est = x_ekf(1);
        y_est = x_ekf(2);
        theta_est = x_ekf(3);
        v_est = x_ekf(4);
        omega_est = x_ekf(5);
        
        %% --- COMPUTE CONTROL (FOLLOW REF PATH) ---
        [v_cmd, omega_cmd, e_y, e_theta, idx_ref] = compute_control(...
            x_est, y_est, theta_est, ref, ...
            cfg.K_ey, cfg.K_etheta, cfg.LOOKAHEAD, cfg.v_nom);
        
        % Current progress along the SAME Dubins path
        dists = sqrt((ref.x - x_est).^2 + (ref.y - y_est).^2);
        [~, idx_closest] = min(dists);
        s_current = ref.s(idx_closest);
        
        % Reference point
        x_ref = ref.x(idx_ref);
        y_ref = ref.y(idx_ref);
        theta_ref = ref.theta(idx_ref);
        
        %% --- SEND COMMAND ---
        sendRobotCmd(sock, v_cmd, omega_cmd, cfg.WHEELBASE);
        
        %% --- CHECK GOAL ---
        dist_to_goal = hypot(x_est - ref.x(end), y_est - ref.y(end));
        if dist_to_goal < 0.2
            goal_reached = true;
        end
        
        %% --- LOG DATA ---
        log.time(end+1)        = t;
        log.x_meas(end+1)      = x_meas;
        log.y_meas(end+1)      = y_meas;
        log.theta_meas(end+1)  = theta_meas;
        log.x_ekf(end+1)       = x_est;
        log.y_ekf(end+1)       = y_est;
        log.theta_ekf(end+1)   = theta_est;
        log.v_ekf(end+1)       = v_est;
        log.omega_ekf(end+1)   = omega_est;
        log.x_ref(end+1)       = x_ref;
        log.y_ref(end+1)       = y_ref;
        log.theta_ref(end+1)   = theta_ref;
        log.e_y(end+1)         = e_y;
        log.e_theta(end+1)     = e_theta;
        log.v_cmd(end+1)       = v_cmd;
        log.omega_cmd(end+1)   = omega_cmd;
        log.s_current(end+1)   = s_current;
        
        %% --- UPDATE VISUALIZATION ---
        trail_x(end+1) = x_est;
        trail_y(end+1) = y_est;
        
        set(h_trail, 'XData', trail_x, 'YData', trail_y);
        set(h_robot, 'XData', x_est, 'YData', y_est);
        set(h_arrow, 'XData', x_est, 'YData', y_est, ...
            'UData', 0.3*cos(theta_est), 'VData', 0.3*sin(theta_est));
        set(h_target, 'XData', x_ref, 'YData', y_ref);
        
        % Update info text
        info_text = sprintf('Time: %.1f s\n\n', t);
        info_text = [info_text sprintf('MEASURED POSE:\n')];
        info_text = [info_text sprintf('  X:     %.3f m\n', x_meas)];
        info_text = [info_text sprintf('  Y:     %.3f m\n', y_meas)];
        info_text = [info_text sprintf('  Theta: %.1f°\n\n', rad2deg(theta_meas))];
        
        info_text = [info_text sprintf('EKF ESTIMATE:\n')];
        info_text = [info_text sprintf('  X:     %.3f m\n', x_est)];
        info_text = [info_text sprintf('  Y:     %.3f m\n', y_est)];
        info_text = [info_text sprintf('  Theta: %.1f°\n', rad2deg(theta_est))];
        info_text = [info_text sprintf('  v:     %.3f m/s\n', v_est)];
        info_text = [info_text sprintf('  omega: %.1f°/s\n\n', rad2deg(omega_est))];
        
        info_text = [info_text sprintf('PROGRESS: %.1f / %.1f m (%.0f%%)\n\n', ...
                            s_current, ref.s(end), 100*s_current/ref.s(end))];
        
        info_text = [info_text sprintf('TRACKING ERRORS:\n')];
        info_text = [info_text sprintf('  Lateral: %.3f m\n', e_y)];
        info_text = [info_text sprintf('  Heading: %.1f°\n\n', rad2deg(e_theta))];
        
        info_text = [info_text sprintf('CONTROL:\n')];
        info_text = [info_text sprintf('  v:     %.3f m/s\n', v_cmd)];
        info_text = [info_text sprintf('  omega: %.1f°/s\n\n', rad2deg(omega_cmd))];
        
        info_text = [info_text sprintf('DISTANCE TO GOAL: %.2f m', dist_to_goal)];
        
        set(h_info, 'String', info_text);
        drawnow limitrate;
        
        % Status print
        if mod(iter, 20) == 0
            fprintf('\r[%.1fs] Pos:(%.2f,%.2f) | Progress:%.0f%% | Errors: e_y=%.3f e_θ=%.1f°', ...
                    t, x_est, y_est, 100*s_current/ref.s(end), e_y, rad2deg(e_theta));
        end
        
        pause(cfg.dt_control);
    end
    
catch ME
    if ~strcmp(ME.identifier, 'MATLAB:interrupt')
        fprintf('\nError: %s\n', ME.message);
    end
    fprintf('\n*** STOPPED ***\n');
end

%% ======================= CLEANUP =======================
fprintf('\n\n=== STOPPING ===\n');

% Stop robot
stopRobot(sock, cfg.WHEELBASE);

if goal_reached
    fprintf('✓ GOAL REACHED!\n');
else
    fprintf('⚠ Did not reach goal (timeout or stopped)\n');
end

% Save logs
log.cfg       = cfg;
log.calib     = calib;
log.ref       = ref;
log.waypoints = waypoints;

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('autonomous_nav_hardcoded_%s.mat', timestamp);
save(filename, 'log');
fprintf('✓ Logs saved to: %s\n', filename);

% Report performance
fprintf('\n=== PERFORMANCE ===\n');
fprintf('Duration: %.1f s\n', log.time(end));
fprintf('Distance traveled: %.2f m\n', log.s_current(end));
fprintf('Mean lateral error: %.3f m\n', mean(abs(log.e_y)));
fprintf('Mean heading error: %.1f°\n', rad2deg(mean(abs(log.e_theta))));
fprintf('Max lateral error: %.3f m\n', max(abs(log.e_y)));
fprintf('Max heading error: %.1f°\n', rad2deg(max(abs(log.e_theta))));

% Cleanup
clear sock;
delete(mq);

fprintf('\n=================================================================\n\n');

%% ======================= UTILITY FUNCTIONS =======================
function [toMazeX, toMazeY] = get_transform_functions(calib)
    switch calib.transform
        case 1
            toMazeX = @(xg, zg) zg - calib.origin_z;
            toMazeY = @(xg, zg) xg - calib.origin_x;
        case 2
            toMazeX = @(xg, zg) -(zg - calib.origin_z);
            toMazeY = @(xg, zg) xg - calib.origin_x;
        case 3
            toMazeX = @(xg, zg) xg - calib.origin_x;
            toMazeY = @(xg, zg) zg - calib.origin_z;
        case 4
            toMazeX = @(xg, zg) xg - calib.origin_x;
            toMazeY = @(xg, zg) -(zg - calib.origin_z);
        otherwise
            error('Invalid transform number: %d', calib.transform);
    end
end

function pose = get_mocap_pose(mq, topic)
    % Read pose from motion capture via MQTT
    pause(0.05);
    tbl = [];
    max_attempts = 50;
    attempts = 0;

    while isempty(tbl) && attempts < max_attempts
        tbl = read(mq, Topic=topic);
        if isempty(tbl)
            pause(0.01);
            attempts = attempts + 1;
        end
    end

    if isempty(tbl)
        error('No MoCap data available');
    end

    data = jsondecode(tbl.Data{end});
    pose.pos = data.pos;

    % Convert quaternion to yaw (rotation about Y-axis for OptiTrack Y-up)
    qx = data.rot(1);
    qy = data.rot(2);
    qz = data.rot(3);
    qw = data.rot(4);

    pose.yaw = atan2(2*(qw*qy - qz*qx), 1 - 2*(qx^2 + qy^2));
end

function [x_maze, y_maze, theta_maze] = transform_pose(pose, calib)
    % Transform MoCap pose to maze coordinate frame
    [toMazeX, toMazeY] = get_transform_functions(calib);
    x_maze = toMazeX(pose.pos(1), pose.pos(3));
    y_maze = toMazeY(pose.pos(1), pose.pos(3));
    theta_maze = wrapToPi(pose.yaw - calib.origin_yaw);
end

function sendRobotCmd(sock, v, omega, wheelbase)
    % Convert (v, omega) to Ackermann steering angle
    % For Ackermann steering: tan(delta) = L * omega / v
    if abs(v) > 0.05
        steering_angle = atan(wheelbase * omega / v);
    else
        steering_angle = omega * 0.3;
    end

    max_steering = deg2rad(30);
    steering_angle = max(min(steering_angle, max_steering), -max_steering);

    cmd = sprintf("%.4f,%.4f\n", v, steering_angle);
    write(sock, cmd);

    if sock.NumBytesAvailable > 0
        response = read(sock, sock.NumBytesAvailable, "string");
    end
end

function stopRobot(sock, wheelbase)
    for i = 1:5
        sendRobotCmd(sock, 0, 0, wheelbase);
        pause(0.05);
    end
end

function [x_pred, P_pred] = ekf_predict(x, P, v_cmd, omega_cmd, dt, Q, tau_v, tau_omega)
    % EKF PREDICTION STEP
    % Propagates the state estimate forward in time using the motion model.
    %
    % Motion Model (Unicycle with first-order velocity dynamics):
    %   x_dot     = v * cos(theta)        % x velocity in world frame
    %   y_dot     = v * sin(theta)        % y velocity in world frame
    %   theta_dot = omega                 % angular velocity
    %   v_dot     = (v_cmd - v) / tau_v   % velocity approaches command
    %   omega_dot = (omega_cmd - omega) / tau_omega
    %
    % The velocity dynamics model accounts for actuator lag using time constants
    % tau_v and tau_omega, making the robot behave as a first-order system.
    %
    % For the EKF, we linearize this nonlinear model by computing the Jacobian
    % F = df/dx evaluated at the current state estimate.

    x = x(:);

    % Extract current state estimates
    theta = x(3);  % Current heading
    v = x(4);      % Current velocity
    omega = x(5);  % Current angular velocity

    % Continuous-time nonlinear dynamics: x_dot = f(x, u)
    f = [v * cos(theta);                  % dx/dt
         v * sin(theta);                  % dy/dt
         omega;                           % dtheta/dt
         (v_cmd - v) / tau_v;             % dv/dt (first-order lag)
         (omega_cmd - omega) / tau_omega]; % domega/dt (first-order lag)

    % Euler integration for discrete-time prediction: x_k+1 = x_k + f*dt
    x_pred = x + f * dt;
    x_pred(3) = wrapToPi(x_pred(3));  % Normalize angle to [-pi, pi]

    % Jacobian of f with respect to state x (linearization for EKF)
    % F_ij = df_i / dx_j
    F = [0, 0, -v*sin(theta), cos(theta), 0;
         0, 0,  v*cos(theta), sin(theta), 0;
         0, 0,  0,            0,          1;
         0, 0,  0,           -1/tau_v,    0;
         0, 0,  0,            0,         -1/tau_omega];

    % Discrete-time state transition matrix: A = I + F*dt
    A = eye(5) + F * dt;

    % Propagate covariance: P_k+1 = A * P_k * A' + Q
    % Covariance grows due to process noise Q
    P_pred = A * P * A' + Q * dt;
end

function [x_upd, P_upd] = ekf_update(x, P, z, R)
    % EKF UPDATE (CORRECTION) STEP
    % Fuses the predicted state with sensor measurements using Kalman gain.
    %
    % Measurement Model:
    %   z = H * x + measurement_noise
    %   where z = [x_meas; y_meas; theta_meas] from motion capture
    %
    % The Kalman gain K optimally weights the innovation (measurement residual)
    % to minimize the posterior covariance. When measurement noise R is small
    % relative to prediction uncertainty P, we trust measurements more.
    %
    % Joseph form for covariance update is numerically more stable than
    % the standard form P = (I - K*H)*P

    x = x(:);
    z = z(:);

    % Measurement model: z = H * x (linear, so H is constant)
    % We only measure position and heading, not velocities
    H = [1, 0, 0, 0, 0;
         0, 1, 0, 0, 0;
         0, 0, 1, 0, 0];

    % Innovation (measurement residual): y = z - h(x_predicted)
    y = z - H * x;
    y(3) = wrapToPi(y(3));  % Handle angle wrapping for heading

    % Innovation covariance: S = H * P * H' + R
    S = H * P * H' + R;

    % Kalman gain: K = P * H' * S^(-1)
    % K determines how much to trust the measurement vs prediction
    K = P * H' / S;

    % Update state estimate: x = x + K * y
    x_upd = x + K * y;
    x_upd(3) = wrapToPi(x_upd(3));

    % Update covariance using Joseph form for numerical stability:
    % P = (I - K*H) * P * (I - K*H)' + K * R * K'
    % This form guarantees P remains symmetric positive definite
    I_KH = eye(5) - K * H;
    P_upd = I_KH * P * I_KH' + K * R * K';
end

function [v_cmd, omega_cmd, e_y, e_theta, idx_ref] = compute_control(x_pos, y_pos, theta_pos, ref, K_ey, K_etheta, lookahead, v_nom)
    % Lateral error feedback controller for path following

    % Find closest point on path
    dists = sqrt((ref.x - x_pos).^2 + (ref.y - y_pos).^2);
    [~, idx_closest] = min(dists);

    % Lookahead improves stability by targeting a point ahead on the path
    s_current = ref.s(idx_closest);
    s_target = min(s_current + lookahead, ref.s(end));
    idx_ref = find(ref.s >= s_target, 1);
    if isempty(idx_ref)
        idx_ref = length(ref.s);
    end

    x_ref = ref.x(idx_ref);
    y_ref = ref.y(idx_ref);
    theta_ref = ref.theta(idx_ref);

    dx = x_pos - x_ref;
    dy = y_pos - y_ref;

    % Cross-track error: perpendicular distance to path (in path frame)
    e_y = -dx * sin(theta_ref) + dy * cos(theta_ref);

    % Heading error
    e_theta = wrapToPi(theta_pos - theta_ref);

    % Proportional control: omega = -(K_ey * e_y + K_etheta * e_theta)
    v_cmd = v_nom;
    omega_cmd = -(K_ey * e_y + K_etheta * e_theta);

    omega_max = 1.0;
    omega_cmd = max(min(omega_cmd, omega_max), -omega_max);
end

function setup_figure_fullscreen()
    figure('Units', 'normalized', 'Position', [0 0 1 1]);
end

function plot_arena(bounds)
    plot([bounds(1) bounds(2) bounds(2) bounds(1) bounds(1)], ...
         [bounds(3) bounds(3) bounds(4) bounds(4) bounds(3)], ...
         'k', 'LineWidth', 3);
end

function plot_obstacles(obs, obsR)
    if isempty(obs)
        return;
    end
    
    scatter(obs(:,1), obs(:,2), 150, 'k', 'filled');
    for i = 1:size(obs, 1)
        viscircles([obs(i,1), obs(i,2)], obsR, ...
                   'Color', [0.8 0 0], 'LineStyle', '--', 'LineWidth', 2);
    end
end

function plot_path(ref, waypoints)
    plot(ref.x, ref.y, 'r--', 'LineWidth', 3);
    plot(waypoints(:,1), waypoints(:,2), 'mo', 'MarkerSize', 15, 'MarkerFaceColor', 'm');
    plot(ref.x(1), ref.y(1), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
end

function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end


function [ref, curvature] = build_dubins_path(wps_2d, R_min, step, v_nom, obs, obsR)
    %% BUILD_DUBINS_PATH - Generate collision-free Dubins path through waypoints
    % 
    % INPUTS:
    %   wps_2d  - Nx2 waypoint positions [x, y]
    %   R_min   - Minimum turning radius [m]
    %   step    - Interpolation step size [m]
    %   v_nom   - Nominal velocity [m/s] for time calculation
    %   obs     - Mx2 obstacle positions [x, y] (optional)
    %   obsR    - Obstacle radius [m] (optional)
    %
    % OUTPUTS:
    %   ref         - Structure with fields:
    %                 .x, .y, .theta (path coordinates)
    %                 .s (arc length)
    %                 .t (time along path)
    %   curvature   - Curvature at each point
    
    %% Check inputs
    if nargin < 5
        obs = [];
        obsR = 0;
    end
    
    %% Helper functions
    dirAngle = @(p1,p2) atan2(p2(2)-p1(2), p2(1)-p1(1));
    
    function pts = dubinsInterp(segObj, step)
        L = segObj.Length;
        s = 0:step:L;
        pts = interpolate(segObj, s);
    end
    
    function tf = collides(path, obs, R)
        tf = false;
        if isempty(obs), return; end
        for i = 1:size(obs,1)
            if any(vecnorm(path(:,1:2)-obs(i,:),2,2) < R)
                tf = true;
                return;
            end
        end
    end
    
    %% Build waypoints with headings
    % Important: Heading at each waypoint points toward NEXT waypoint
    wps = zeros(size(wps_2d,1), 3);
    for k = 1:size(wps_2d,1)
        wps(k,1:2) = wps_2d(k,:);
        if k < size(wps_2d,1)
            % Point toward next waypoint
            wps(k,3) = dirAngle(wps_2d(k,:), wps_2d(k+1,:));
        else
            % Last waypoint: keep previous heading
            wps(k,3) = wps(k-1,3);
        end
    end
    
    %% Connect waypoints with Dubins curves
    dubConn = dubinsConnection;
    dubConn.MinTurningRadius = R_min;
    fullPath = [];
    
    for k = 1:(size(wps,1)-1)
        p1 = wps(k,:);
        p2 = wps(k+1,:);
        
        % Get all possible Dubins paths (LSL, RSR, LSR, RSL, RLR, LRL)
        [segList, costs] = connect(dubConn, p1, p2, "PathSegments", "all");
        
        % Find shortest collision-free path
        found_safe = false;
        [sorted_costs, sort_idx] = sort(costs);
        
        for i = 1:length(segList)
            idx = sort_idx(i);
            pts = dubinsInterp(segList{idx}, step);
            
            % Check collision with actual obstacle radius
            if ~collides(pts, obs, obsR)
                fullPath = [fullPath; pts];
                found_safe = true;
                break;
            end
        end
        
        if ~found_safe
            error('No collision-free Dubins path from WP %d to %d. A* waypoints too close to obstacles.', k, k+1);
        end
    end
    
    %% Extract coordinates
    x = fullPath(:,1);
    y = fullPath(:,2);
    theta = fullPath(:,3);
    
    %% Compute arc length
    dx = diff(x);
    dy = diff(y);
    ds = sqrt(dx.^2 + dy.^2);
    s = [0; cumsum(ds)];
    
    %% Compute time (assuming constant velocity)
    t = s / v_nom;
    
    %% Compute curvature (κ = dθ/ds)
    dtheta = diff(theta);
    dtheta = wrapToPi(dtheta);  % Handle angle wrapping
    kappa = zeros(size(theta));
    kappa(2:end) = dtheta ./ max(ds, 1e-6);
    kappa(1) = kappa(2);  % Copy first valid curvature to first point
    
    % Smooth curvature to remove numerical spikes
    if length(kappa) > 5
        kappa = movmean(kappa, 5);
    end
    
    %% Package output
    ref.x = x;
    ref.y = y;
    ref.theta = theta;
    ref.s = s;
    ref.t = t;
    curvature = kappa;
    
end
