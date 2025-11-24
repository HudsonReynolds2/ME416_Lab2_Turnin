function sim_ekf_lqr_dubins()
    %SIM_EKF_LQR_DUBINS Full simulation of path following controller
    %
    % This simulates the complete system:
    %   - Limo robot dynamics
    %   - MoCap feedback with noise
    %   - EKF state estimation
    %   - Dubins path planning
    %   - LQR path tracking
    %
    % Usage:
    %   sim_ekf_lqr_dubins()  % Run with default settings
    
    close all; clc;
    
    fprintf('=================================================================\n');
    fprintf('FULL PATH FOLLOWING SIMULATION\n');
    fprintf('=================================================================\n\n');
    
    %% Configuration
    % Choose which error formulation to test
    % Based on test_control_signs_with_simulator results
    USE_CONFIG = 'auto';  % 'auto' will use the one that worked in tests
                          % Or specify config number (1-16)
    
    %% Waypoints (same as your lab)
    wps_2d = [
        0.0, 0.0;   % Start
        0.8, 0.2;   % WP1
        1.3, 0.2;   % WP2
        1.3, -0.3;  % WP3
        1.8, -0.3;  % WP4
        1.8, 0.2;   % WP5
        2.5, 0.2    % Goal
    ];
    
    fprintf('Waypoints:\n');
    for i = 1:size(wps_2d, 1)
        fprintf('  WP%d: (%.1f, %.1f)\n', i-1, wps_2d(i,1), wps_2d(i,2));
    end
    fprintf('\n');
    
    %% Generate Dubins path
    fprintf('Generating Dubins path...\n');
    
    % Compute headings between waypoints
    headings = zeros(size(wps_2d, 1), 1);
    for i = 1:size(wps_2d, 1)-1
        dx = wps_2d(i+1, 1) - wps_2d(i, 1);
        dy = wps_2d(i+1, 2) - wps_2d(i, 2);
        headings(i) = atan2(dy, dx);
    end
    headings(end) = headings(end-1);  % Last heading same as second-to-last
    
    % Create 3D waypoints with headings
    wps_3d = [wps_2d, headings];
    
    % Generate Dubins path
    turning_radius = 0.3;  % meters
    path = generate_dubins_path(wps_3d, turning_radius);
    
    fprintf('Path generated: %.2f meters total length\n', path.s(end));
    fprintf('Number of path points: %d\n\n', length(path.s));
    
    %% Load control configuration
    if strcmp(USE_CONFIG, 'auto')
        % Try to load from previous test
        if exist('sign_test_results.mat', 'file')
            load('sign_test_results.mat', 'results');
            scores = [results.metrics.score];
            [~, best_idx] = max(scores);
            config = results(best_idx).config;
            fprintf('Using configuration #%d from previous test\n', best_idx);
        else
            fprintf('No previous test results found.\n');
            fprintf('Using Configuration #6 (validated best: 0.14m error)\n');
            config = get_default_config();
        end
    else
        load('sign_test_results.mat', 'results');
        config = results(USE_CONFIG).config;
        fprintf('Using user-specified configuration #%d\n', USE_CONFIG);
    end
    
    fprintf('\nControl configuration:\n');
    fprintf('  %s\n', config.ey.name);
    fprintf('  %s\n', config.etheta.name);
    fprintf('  %s\n\n', config.control.name);
    
    %% Initialize simulator
    sim = LimoSimulator();
    sim.setCalibration(4);  % Use Transform 4 (verified correct)
    sim.reset([wps_2d(1,1), wps_2d(1,2), headings(1)]);
    
    %% Initialize EKF
    % State: [x, y, theta, v]
    x_est = [wps_2d(1,1); wps_2d(1,2); headings(1); 0];
    
    % Initial covariance
    P = diag([0.01, 0.01, deg2rad(5), 0.01].^2);
    
    % Process noise
    Q = diag([0.02, 0.02, deg2rad(2), 0.05].^2);
    
    % Measurement noise (MoCap)
    R = diag([0.005, 0.005, deg2rad(1)].^2);
    
    fprintf('EKF initialized\n');
    fprintf('  Initial state: x=%.3f, y=%.3f, θ=%.1f°, v=%.2f\n', ...
            x_est(1), x_est(2), rad2deg(x_est(3)), x_est(4));
    fprintf('\n');
    
    %% Controller parameters
    K_lqr = [8.0, 3.0, 0.5];  % [ey, etheta, ev] gains
    LOOKAHEAD_DIST = 0.4;     % meters
    V_DESIRED = 0.2;          % m/s
    
    %% Simulation loop
    dt = 0.05;
    T_final = 50.0;  % Run for up to 50 seconds
    
    % Storage
    history = struct();
    history.t = [];
    history.x_true = []; history.y_true = []; history.theta_true = [];
    history.x_est = []; history.y_est = []; history.theta_est = [];
    history.x_ref = []; history.y_ref = []; history.theta_ref = [];
    history.ey = []; history.etheta = [];
    history.v_cmd = []; history.omega_cmd = [];
    history.s_current = [];
    
    fprintf('Starting simulation...\n');
    fprintf('Time | Position (est) | Heading | Errors | Control\n');
    fprintf('-----|----------------|---------|--------|--------\n');
    
    s_current = 0;  % Current arc length along path
    
    for t = 0:dt:T_final
        %% Get true state from simulator
        [x_true, y_true, theta_true] = sim.getState();
        
        %% Simulate MoCap measurement
        mocap = sim.getMocapPose(true);  % With noise
        
        % Transform MoCap to maze frame (simulate what controller does)
        [x_meas, y_meas, theta_meas] = transform_mocap_to_maze(mocap, sim.calib);
        z_meas = [x_meas; y_meas; theta_meas];
        
        %% EKF Update
        % Measurement model: H = [1 0 0 0; 0 1 0 0; 0 0 1 0]
        H = [eye(3), zeros(3,1)];
        
        % Innovation
        y_innov = z_meas - H * x_est;
        y_innov(3) = wrapToPi(y_innov(3));  % Wrap angle
        
        % Kalman gain
        S = H * P * H' + R;
        K = P * H' / S;
        
        % Update
        x_est = x_est + K * y_innov;
        x_est(3) = wrapToPi(x_est(3));
        P = (eye(4) - K * H) * P;
        
        %% Find reference point on path
        % Update arc length estimate
        v_est = x_est(4);
        s_current = s_current + v_est * dt;
        s_current = max(0, min(path.s(end), s_current));
        
        % Find reference point ahead
        s_ref = s_current + LOOKAHEAD_DIST;
        s_ref = min(s_ref, path.s(end));
        
        % Interpolate path at s_ref
        idx_ref = find(path.s >= s_ref, 1, 'first');
        if isempty(idx_ref), idx_ref = length(path.s); end
        
        x_ref = path.x(idx_ref);
        y_ref = path.y(idx_ref);
        theta_ref = path.theta(idx_ref);
        
        %% Compute errors
        x_e = x_est(1);
        y_e = x_est(2);
        theta_e = x_est(3);
        
        dx = x_e - x_ref;
        dy = y_e - y_ref;
        
        % Use configured error formulas
        ey = config.ey.func(dx, dy, theta_ref);
        etheta = config.etheta.func(theta_ref, theta_e);
        ev = v_est - V_DESIRED;
        
        %% LQR control
        omega_fb = config.control.sign * (K_lqr(1)*ey + K_lqr(2)*etheta + K_lqr(3)*ev);
        
        % Feedforward (curvature)
        if idx_ref < length(path.s)
            ds = path.s(idx_ref+1) - path.s(idx_ref);
            dtheta = wrapToPi(path.theta(idx_ref+1) - path.theta(idx_ref));
            kappa = dtheta / max(ds, 0.01);
            omega_ff = v_est * kappa;
        else
            omega_ff = 0;
        end
        
        omega_total = omega_fb + omega_ff;
        
        % Saturate
        v_cmd = V_DESIRED;
        omega_cmd = max(-1.0, min(1.0, omega_total));
        
        %% EKF Predict (using commanded inputs)
        % Motion model: unicycle
        F = [
            1, 0, -v_cmd*sin(x_est(3))*dt, cos(x_est(3))*dt;
            0, 1,  v_cmd*cos(x_est(3))*dt, sin(x_est(3))*dt;
            0, 0, 1, 0;
            0, 0, 0, 0.95  % Velocity damping
        ];
        
        B = [
            cos(x_est(3))*dt, 0;
            sin(x_est(3))*dt, 0;
            0, dt;
            1, 0
        ];
        
        u = [v_cmd; omega_cmd];
        
        x_est = F * x_est + B * u;
        x_est(3) = wrapToPi(x_est(3));
        P = F * P * F' + Q;
        
        %% Step simulator
        sim.step(v_cmd, omega_cmd, dt);
        
        %% Record history
        history.t(end+1) = t;
        history.x_true(end+1) = x_true;
        history.y_true(end+1) = y_true;
        history.theta_true(end+1) = theta_true;
        history.x_est(end+1) = x_est(1);
        history.y_est(end+1) = x_est(2);
        history.theta_est(end+1) = x_est(3);
        history.x_ref(end+1) = x_ref;
        history.y_ref(end+1) = y_ref;
        history.theta_ref(end+1) = theta_ref;
        history.ey(end+1) = ey;
        history.etheta(end+1) = etheta;
        history.v_cmd(end+1) = v_cmd;
        history.omega_cmd(end+1) = omega_cmd;
        history.s_current(end+1) = s_current;
        
        %% Print progress
        if mod(round(t/dt), 20) == 0  % Every second
            fprintf('%4.1fs | (%.2f, %.2f) | %+5.1f° | ey=%+.3f eθ=%+.1f° | ω=%+.2f\n', ...
                    t, x_est(1), x_est(2), rad2deg(x_est(3)), ...
                    ey, rad2deg(etheta), omega_cmd);
        end
        
        %% Check if reached goal
        dist_to_goal = hypot(x_est(1) - wps_2d(end,1), x_est(2) - wps_2d(end,2));
        if dist_to_goal < 0.1 && s_current > path.s(end) - 0.2
            fprintf('\nGoal reached at t=%.1fs!\n', t);
            break;
        end
    end
    
    fprintf('\nSimulation complete!\n\n');
    
    %% Analyze results
    fprintf('=================================================================\n');
    fprintf('PERFORMANCE METRICS\n');
    fprintf('=================================================================\n');
    
    % Cross-track error
    ey_rms = sqrt(mean(history.ey.^2));
    ey_max = max(abs(history.ey));
    fprintf('Cross-track error (ey):\n');
    fprintf('  RMS: %.3f m\n', ey_rms);
    fprintf('  Max: %.3f m\n', ey_max);
    
    % Heading error
    etheta_rms = sqrt(mean(history.etheta.^2));
    etheta_max = max(abs(history.etheta));
    fprintf('Heading error (eθ):\n');
    fprintf('  RMS: %.1f°\n', rad2deg(etheta_rms));
    fprintf('  Max: %.1f°\n', rad2deg(etheta_max));
    
    % Final position
    fprintf('Final position:\n');
    fprintf('  Actual: (%.3f, %.3f)\n', history.x_true(end), history.y_true(end));
    fprintf('  Target: (%.3f, %.3f)\n', wps_2d(end,1), wps_2d(end,2));
    final_error = hypot(history.x_true(end) - wps_2d(end,1), ...
                        history.y_true(end) - wps_2d(end,2));
    fprintf('  Error: %.3f m\n', final_error);
    
    %% Plot results
    plot_simulation_results(history, path, wps_2d, config);
    
    fprintf('\n=== Simulation complete! ===\n');
end

%% Helper functions

function path = generate_dubins_path(waypoints_3d, radius)
    % Generate Dubins path through waypoints
    % Simplified version - just connect with straight lines and circles
    
    path.x = [];
    path.y = [];
    path.theta = [];
    path.s = [];
    
    s = 0;
    ds = 0.01;  % 1cm resolution
    
    for i = 1:size(waypoints_3d, 1)-1
        x1 = waypoints_3d(i, 1);
        y1 = waypoints_3d(i, 2);
        theta1 = waypoints_3d(i, 3);
        
        x2 = waypoints_3d(i+1, 1);
        y2 = waypoints_3d(i+1, 2);
        theta2 = waypoints_3d(i+1, 3);
        
        % Simple: just interpolate linearly
        dist = hypot(x2 - x1, y2 - y1);
        n_points = ceil(dist / ds);
        
        for j = 0:n_points
            alpha = j / n_points;
            x = x1 + alpha * (x2 - x1);
            y = y1 + alpha * (y2 - y1);
            theta = theta1 + alpha * wrapToPi(theta2 - theta1);
            
            path.x(end+1) = x;
            path.y(end+1) = y;
            path.theta(end+1) = theta;
            path.s(end+1) = s;
            
            if j < n_points
                s = s + ds;
            end
        end
    end
end

function [x_maze, y_maze, theta_maze] = transform_mocap_to_maze(mocap, calib)
    % Transform MoCap global to maze frame
    
    x_global = mocap.pos(1);
    z_global = mocap.pos(3);
    
    % Extract yaw from quaternion
    qx = mocap.rot(1);
    qy = mocap.rot(2);
    qz = mocap.rot(3);
    qw = mocap.rot(4);
    yaw_global = atan2(2*(qw*qy + qx*qz), 1 - 2*(qy^2 + qz^2));
    
    % Apply transform
    switch calib.transform
        case 1
            x_maze = z_global - calib.origin_z;
            y_maze = x_global - calib.origin_x;
        case 2
            x_maze = -(z_global - calib.origin_z);
            y_maze = x_global - calib.origin_x;
        case 3
            x_maze = x_global - calib.origin_x;
            y_maze = z_global - calib.origin_z;
        case 4
            x_maze = x_global - calib.origin_x;
            y_maze = -(z_global - calib.origin_z);
        case 5
            x_maze = -(x_global - calib.origin_x);
            y_maze = z_global - calib.origin_z;
        case 6
            x_maze = -(x_global - calib.origin_x);
            y_maze = -(z_global - calib.origin_z);
        case 7
            x_maze = z_global - calib.origin_z;
            y_maze = -(x_global - calib.origin_x);
        case 8
            x_maze = -(z_global - calib.origin_z);
            y_maze = -(x_global - calib.origin_x);
    end
    
    theta_maze = wrapToPi(yaw_global - calib.origin_yaw);
end

function config = get_default_config()
    % Config #6 - validated as best (0.14m error to target)
    config.ey.name = 'ey = +dx*sin(θ) - dy*cos(θ)';
    config.ey.func = @(dx, dy, th) dx*sin(th) - dy*cos(th);
    config.etheta.name = 'eθ = θ_ref - θ';
    config.etheta.func = @(th_ref, th) wrapToPi(th_ref - th);
    config.control.name = 'ω = +(K·e)';
    config.control.sign = +1;
    config.id = 6;
end

function plot_simulation_results(history, path, waypoints, config)
    % Plot comprehensive results
    
    figure('Name', 'Simulation Results', 'Position', [50, 50, 1600, 1000]);
    
    % Trajectory
    subplot(2, 4, [1, 2, 5, 6]);
    hold on; grid on; axis equal;
    
    % Reference path
    plot(path.x, path.y, 'r--', 'LineWidth', 2, 'DisplayName', 'Reference Path');
    
    % Waypoints
    plot(waypoints(:,1), waypoints(:,2), 'ko', 'MarkerSize', 8, ...
         'MarkerFaceColor', 'y', 'DisplayName', 'Waypoints');
    
    % Actual trajectory (true)
    plot(history.x_true, history.y_true, 'b-', 'LineWidth', 2, 'DisplayName', 'True Path');
    
    % Estimated trajectory
    plot(history.x_est, history.y_est, 'g:', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
    
    % Start/end markers
    plot(history.x_true(1), history.y_true(1), 'go', 'MarkerSize', 12, ...
         'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot(history.x_true(end), history.y_true(end), 'ro', 'MarkerSize', 12, ...
         'MarkerFaceColor', 'r', 'DisplayName', 'End');
    
    % Coordinate axes
    quiver(0, 0, 0.3, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    text(0.32, 0, '+X', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
    quiver(0, 0, 0, 0.3, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    text(0, 0.32, '+Y', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
    
    xlabel('X (m)'); ylabel('Y (m)');
    title('Trajectory');
    legend('Location', 'best');
    xlim([-0.5, max(waypoints(:,1))+0.5]);
    ylim([min(waypoints(:,2))-0.5, max(waypoints(:,2))+0.5]);
    
    % Position errors
    subplot(2, 4, 3);
    plot(history.t, history.ey, 'b-', 'LineWidth', 1.5);
    hold on; yline(0, 'k--', 'Alpha', 0.3);
    grid on;
    xlabel('Time (s)'); ylabel('e_y (m)');
    title('Cross-track Error');
    
    subplot(2, 4, 4);
    plot(history.t, rad2deg(history.etheta), 'r-', 'LineWidth', 1.5);
    hold on; yline(0, 'k--', 'Alpha', 0.3);
    grid on;
    xlabel('Time (s)'); ylabel('e_θ (deg)');
    title('Heading Error');
    
    % Control outputs
    subplot(2, 4, 7);
    plot(history.t, history.v_cmd, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)'); ylabel('v (m/s)');
    title('Linear Velocity');
    ylim([0, 0.3]);
    
    subplot(2, 4, 8);
    plot(history.t, history.omega_cmd, 'r-', 'LineWidth', 1.5);
    hold on; yline(0, 'k--', 'Alpha', 0.3);
    grid on;
    xlabel('Time (s)'); ylabel('ω (rad/s)');
    title('Angular Velocity');
    
    % Add configuration info
    sgtitle({sprintf('Path Following Simulation (t=%.1fs)', history.t(end)), ...
             config.ey.name, config.etheta.name, config.control.name}, ...
            'FontSize', 10);
end
