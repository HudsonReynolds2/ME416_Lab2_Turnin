function results = test_control_signs_with_simulator()
    %TEST_CONTROL_SIGNS_WITH_SIMULATOR Automatically test all control sign combinations
    %
    % This runs a simulated robot with different error formulations to find
    % the correct one. The robot should move from origin toward (0.8, 0.2).
    %
    % Returns: results structure with best configuration
    
    close all;
    
    fprintf('=================================================================\n');
    fprintf('AUTOMATED CONTROL SIGN TESTING WITH SIMULATOR\n');
    fprintf('=================================================================\n\n');
    fprintf('Testing scenario:\n');
    fprintf('  Start: (0, 0) @ 14°\n');
    fprintf('  Target: Move toward waypoint (0.8, 0.2)\n');
    fprintf('  Expected: Y should increase (robot moves UP)\n\n');
    
    %% Define all possible formulations
    test_configs = {};
    config_id = 1;
    
    % Cross-track error formulas
    ey_funcs = {
        struct('name', 'ey = -dx*sin(θ) + dy*cos(θ)', ...
               'func', @(dx, dy, th) -dx*sin(th) + dy*cos(th)), ...
        struct('name', 'ey = +dx*sin(θ) - dy*cos(θ)', ...
               'func', @(dx, dy, th) dx*sin(th) - dy*cos(th)), ...
        struct('name', 'ey = +dx*sin(θ) + dy*cos(θ)', ...
               'func', @(dx, dy, th) dx*sin(th) + dy*cos(th)), ...
        struct('name', 'ey = -dx*sin(θ) - dy*cos(θ)', ...
               'func', @(dx, dy, th) -dx*sin(th) - dy*cos(th))
    };
    
    % Heading error formulas
    etheta_funcs = {
        struct('name', 'eθ = θ_ref - θ', ...
               'func', @(th_ref, th) wrapToPi(th_ref - th)), ...
        struct('name', 'eθ = θ - θ_ref', ...
               'func', @(th_ref, th) wrapToPi(th - th_ref))
    };
    
    % Control law signs
    control_funcs = {
        struct('name', 'ω = -(K·e)', 'sign', -1), ...
        struct('name', 'ω = +(K·e)', 'sign', +1)
    };
    
    % Generate all combinations
    for i = 1:length(ey_funcs)
        for j = 1:length(etheta_funcs)
            for k = 1:length(control_funcs)
                test_configs{config_id} = struct(...
                    'id', config_id, ...
                    'ey', ey_funcs{i}, ...
                    'etheta', etheta_funcs{j}, ...
                    'control', control_funcs{k});
                config_id = config_id + 1;
            end
        end
    end
    
    fprintf('Total configurations to test: %d\n\n', length(test_configs));
    
    %% Run simulations for each configuration
    results = struct('configs', {}, 'metrics', {});
    
    for idx = 1:length(test_configs)
        cfg = test_configs{idx};
        
        fprintf('Testing configuration #%d/%d...\n', idx, length(test_configs));
        
        % Run simulation
        metrics = run_single_test(cfg);
        
        % Store results
        results(idx).config = cfg;
        results(idx).metrics = metrics;
        
        % Print summary
        fprintf('  Final: (%.3f, %.3f) | Err to target: %.3f m | Score: %.3f', ...
                metrics.final_x, metrics.final_y, metrics.final_error, metrics.score);
        
        if metrics.is_correct
            fprintf('  ✓ CORRECT\n');
        else
            fprintf('  ✗ WRONG\n');
        end
    end
    
    %% Find best configuration
    fprintf('\n=================================================================\n');
    fprintf('ANALYSIS\n');
    fprintf('=================================================================\n\n');
    
    % Extract scores properly
    scores = zeros(1, length(results));
    for i = 1:length(results)
        scores(i) = results(i).metrics.score;
    end
    [~, best_idx] = max(scores);
    
    % Find all "correct" configurations
    correct_indices = [];
    for i = 1:length(results)
        if results(i).metrics.is_correct
            correct_indices(end+1) = i;
        end
    end
    
    fprintf('Configurations that produce positive Y movement:\n');
    for i = 1:length(correct_indices)
        idx = correct_indices(i);
        cfg = results(idx).config;
        m = results(idx).metrics;
        fprintf('\n#%d (Score: %.3f, Error: %.3f m):\n', idx, m.score, m.final_error);
        fprintf('  %s\n', cfg.ey.name);
        fprintf('  %s\n', cfg.etheta.name);
        fprintf('  %s\n', cfg.control.name);
        fprintf('  → Final position: (%.3f, %.3f)\n', m.final_x, m.final_y);
    end
    
    if ~isempty(correct_indices)
        fprintf('\n=================================================================\n');
        fprintf('RECOMMENDED CONFIGURATION: #%d\n', best_idx);
        fprintf('=================================================================\n');
        best_cfg = results(best_idx).config;
        fprintf('\nUse this in your controller:\n\n');
        fprintf('  // Cross-track error\n');
        fprintf('  %s\n\n', best_cfg.ey.name);
        fprintf('  // Heading error\n');
        fprintf('  %s\n\n', best_cfg.etheta.name);
        fprintf('  // Control law\n');
        fprintf('  %s\n\n', best_cfg.control.name);
        
        % Show detailed trajectory
        fprintf('Running detailed simulation with best configuration...\n');
        visualize_configuration(best_cfg);
    else
        fprintf('\nWARNING: No configuration produced correct behavior!\n');
        fprintf('Check your LQR gains or test scenario.\n');
    end
    
    % Save results
    save('sign_test_results.mat', 'results', 'test_configs');
    fprintf('\nResults saved to: sign_test_results.mat\n');
end

%% Helper function: Run single test
function metrics = run_single_test(config)
    % Run a 5-second simulation with given configuration
    
    % Create simulator
    sim = LimoSimulator();
    sim.reset([0, 0, deg2rad(14)]);  % Start at origin, heading toward (0.8, 0.2)
    
    % Simple reference path (straight line toward waypoint)
    waypoint = [0.8, 0.2];
    
    % LQR gains (same as your main code)
    K_lqr = [8.0, 3.0, 0.5];  % [ey, etheta, ev]
    
    % Simulation parameters
    dt = 0.05;
    T_final = 5.0;
    v_cmd = 0.2;  % constant velocity
    
    % Run simulation
    for t = 0:dt:T_final
        % Get current state
        [x, y, theta] = sim.getState();
        
        % Compute reference point (simple: just point toward waypoint)
        dx_to_wp = waypoint(1) - x;
        dy_to_wp = waypoint(2) - y;
        theta_ref = atan2(dy_to_wp, dx_to_wp);
        
        % Use a point ahead on the line to waypoint
        lookahead = 0.4;
        dist_to_wp = hypot(dx_to_wp, dy_to_wp);
        if dist_to_wp > lookahead
            s = lookahead / dist_to_wp;
            x_ref = x + s * dx_to_wp;
            y_ref = y + s * dy_to_wp;
        else
            x_ref = waypoint(1);
            y_ref = waypoint(2);
        end
        
        % Position errors
        dx = x - x_ref;
        dy = y - y_ref;
        
        % Compute errors using this configuration
        ey = config.ey.func(dx, dy, theta_ref);
        etheta = config.etheta.func(theta_ref, theta);
        ev = 0;  % Not using velocity error for this test
        
        % Compute control
        omega = config.control.sign * (K_lqr(1)*ey + K_lqr(2)*etheta + K_lqr(3)*ev);
        
        % Saturate
        omega = max(-1.0, min(1.0, omega));
        
        % Step simulation
        sim.step(v_cmd, omega, dt);
    end
    
    % Compute metrics
    [final_x, final_y, ~] = sim.getState();
    max_y = max(sim.history.y);
    min_y = min(sim.history.y);
    
    % Score based on distance to target waypoint (0.8, 0.2)
    target_x = waypoint(1);
    target_y = waypoint(2);
    final_error = hypot(final_x - target_x, final_y - target_y);
    
    % Lower error = higher score
    % Also reward positive Y movement
    score = -final_error + 0.5 * (final_y > 0);
    
    % Consider "correct" if:
    % 1. Y increases overall (moves upward)
    % 2. Gets reasonably close to target (within 0.5m)
    is_correct = (final_y > 0.05) && (final_error < 0.5);
    
    metrics = struct(...
        'final_x', final_x, ...
        'final_y', final_y, ...
        'max_y', max_y, ...
        'min_y', min_y, ...
        'final_error', final_error, ...
        'score', score, ...
        'is_correct', is_correct);
end

%% Helper function: Visualize best configuration
function visualize_configuration(config)
    % Run detailed simulation and plot
    
    % Create simulator
    sim = LimoSimulator();
    sim.reset([0, 0, deg2rad(14)]);
    
    % Reference path
    waypoint = [0.8, 0.2];
    
    % LQR gains
    K_lqr = [8.0, 3.0, 0.5];
    
    % Simulation
    dt = 0.05;
    T_final = 10.0;
    v_cmd = 0.2;
    
    % Storage for detailed analysis
    history = struct('t', [], 'x', [], 'y', [], 'theta', [], ...
                     'ey', [], 'etheta', [], 'omega', [], ...
                     'x_ref', [], 'y_ref', [], 'theta_ref', []);
    
    for t = 0:dt:T_final
        [x, y, theta] = sim.getState();
        
        % Reference point
        dx_to_wp = waypoint(1) - x;
        dy_to_wp = waypoint(2) - y;
        theta_ref = atan2(dy_to_wp, dx_to_wp);
        
        lookahead = 0.4;
        dist_to_wp = hypot(dx_to_wp, dy_to_wp);
        if dist_to_wp > lookahead
            s = lookahead / dist_to_wp;
            x_ref = x + s * dx_to_wp;
            y_ref = y + s * dy_to_wp;
        else
            x_ref = waypoint(1);
            y_ref = waypoint(2);
        end
        
        % Errors
        dx = x - x_ref;
        dy = y - y_ref;
        ey = config.ey.func(dx, dy, theta_ref);
        etheta = config.etheta.func(theta_ref, theta);
        
        % Control
        omega = config.control.sign * (K_lqr(1)*ey + K_lqr(2)*etheta);
        omega = max(-1.0, min(1.0, omega));
        
        % Record
        history.t(end+1) = t;
        history.x(end+1) = x;
        history.y(end+1) = y;
        history.theta(end+1) = theta;
        history.ey(end+1) = ey;
        history.etheta(end+1) = etheta;
        history.omega(end+1) = omega;
        history.x_ref(end+1) = x_ref;
        history.y_ref(end+1) = y_ref;
        history.theta_ref(end+1) = theta_ref;
        
        % Step
        sim.step(v_cmd, omega, dt);
        
        % Stop if reached waypoint
        if dist_to_wp < 0.1
            break;
        end
    end
    
    % Plot detailed results
    figure('Name', 'Best Configuration', 'Position', [100, 100, 1400, 900]);
    
    % Trajectory
    subplot(2, 3, [1, 4]);
    hold on; grid on; axis equal;
    plot([0, waypoint(1)], [0, waypoint(2)], 'r--', 'LineWidth', 2, 'DisplayName', 'Target');
    plot(history.x, history.y, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
    plot(0, 0, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot(waypoint(1), waypoint(2), 'r^', 'MarkerSize', 12, 'MarkerFaceColor', 'r', ...
         'DisplayName', 'Waypoint');
    plot(history.x(end), history.y(end), 'bo', 'MarkerSize', 10, ...
         'MarkerFaceColor', 'b', 'DisplayName', 'End');
    
    % Arrows
    quiver(0, 0, 0.3, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    text(0.3, -0.05, '+X', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
    quiver(0, 0, 0, 0.3, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    text(-0.05, 0.3, '+Y', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
    
    xlabel('X (m)'); ylabel('Y (m)');
    title('Trajectory');
    legend('Location', 'best');
    xlim([-0.2, 1.0]); ylim([-0.3, 0.5]);
    
    % X vs time
    subplot(2, 3, 2);
    plot(history.t, history.x, 'b-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('X (m)'); title('X Position');
    
    % Y vs time
    subplot(2, 3, 3);
    plot(history.t, history.y, 'b-', 'LineWidth', 1.5);
    hold on; yline(0, 'r--', 'LineWidth', 1);
    grid on; xlabel('Time (s)'); ylabel('Y (m)'); title('Y Position');
    if all(history.y >= -0.05)
        text(mean(history.t), max(history.y)*0.8, '✓ Y always positive', ...
             'FontSize', 12, 'Color', 'g', 'FontWeight', 'bold');
    end
    
    % Errors
    subplot(2, 3, 5);
    yyaxis left
    plot(history.t, history.ey, 'b-', 'LineWidth', 1.5);
    ylabel('e_y (m)'); grid on;
    yyaxis right
    plot(history.t, rad2deg(history.etheta), 'r-', 'LineWidth', 1.5);
    ylabel('e_θ (deg)');
    xlabel('Time (s)'); title('Tracking Errors');
    legend('e_y', 'e_θ');
    
    % Control
    subplot(2, 3, 6);
    plot(history.t, history.omega, 'r-', 'LineWidth', 1.5);
    hold on; yline(0, 'k--', 'Alpha', 0.3);
    grid on; xlabel('Time (s)'); ylabel('ω (rad/s)'); title('Angular Velocity Command');
    
    % Add configuration info
    sgtitle({sprintf('Best Configuration (#%d)', config.id), ...
             config.ey.name, config.etheta.name, config.control.name}, ...
            'FontSize', 11);
end
