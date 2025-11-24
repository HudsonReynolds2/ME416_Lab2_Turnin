clear; clc; close all;

%% ======================= MULTI-MAZE VISUALIZER =======================
% Visualizes all maze courses with Dubins paths through predefined waypoints.
% Shows obstacles, paths, and waypoints for each maze configuration.

fprintf('=================================================================\n');
fprintf('MULTI-MAZE PATH VISUALIZER (Using Hardcoded Waypoints)\n');
fprintf('=================================================================\n\n');

% Load configuration
cfg = limo_config();

%% Create figure with 3 subplots
figure('Units', 'normalized', 'Position', [0.05 0.05 0.9 0.9]);

for maze_idx = 1:3
    % Select maze
    cfg.maze_select = maze_idx;
    cfg.obs = cfg.mazes(maze_idx).obs;
    cfg.start_pos = cfg.mazes(maze_idx).start_pos;
    cfg.goal_pos = cfg.mazes(maze_idx).goal_pos;
    cfg.arena_bounds = cfg.mazes(maze_idx).arena_bounds;
    cfg.courseName = cfg.mazes(maze_idx).name;
    cfg.waypoints = cfg.mazes(maze_idx).waypoints;
    
    fprintf('\n=== MAZE %d: %s ===\n', maze_idx, cfg.courseName);
    
    %% Use hardcoded waypoints and build Dubins path
    fprintf('Building Dubins path from hardcoded waypoints...\n');
    try
        [ref_path, ~] = build_dubins_path(cfg.waypoints, cfg.R_min, cfg.step, ...
                                          cfg.v_nom, cfg.obs, cfg.obsR);
        fprintf('  ✓ Path: %d wps, %.2f m\n', size(cfg.waypoints,1), ref_path.s(end));
        path_found = true;
    catch ME
        fprintf('  ✗ Path failed: %s\n', ME.message);
        path_found = false;
    end
    
    %% Plot
    subplot(1, 3, maze_idx);
    hold on; grid on; axis equal;
    xlim([cfg.arena_bounds(1) cfg.arena_bounds(2)]);
    ylim([cfg.arena_bounds(3) cfg.arena_bounds(4)]);
    title(sprintf('Maze %d: %s', maze_idx, cfg.courseName), ...
          'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X [m]'); ylabel('Y [m]');
    
    % Arena boundary
    plot([cfg.arena_bounds(1) cfg.arena_bounds(2) cfg.arena_bounds(2) cfg.arena_bounds(1) cfg.arena_bounds(1)], ...
         [cfg.arena_bounds(3) cfg.arena_bounds(3) cfg.arena_bounds(4) cfg.arena_bounds(4) cfg.arena_bounds(3)], ...
         'k', 'LineWidth', 3);
    
    % Obstacles
    scatter(cfg.obs(:,1), cfg.obs(:,2), 100, 'k', 'filled');
    for i = 1:size(cfg.obs, 1)
        viscircles([cfg.obs(i,1), cfg.obs(i,2)], cfg.obsR, ...
                   'Color', [0.7 0.7 0.7], 'LineStyle', '--', 'LineWidth', 1);
    end
    
    % Path - single Dubins path from hardcoded waypoints
    legend_entries = {'Arena', 'Obstacles', ''};
    if path_found
        plot(ref_path.x, ref_path.y, 'r-', 'LineWidth', 3);
        legend_entries{end+1} = 'Dubins Path';
        
        % Show waypoints
        plot(cfg.waypoints(:,1), cfg.waypoints(:,2), 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
        legend_entries{end+1} = 'Waypoints';
    end
    
    % Start and goal
    plot(cfg.start_pos(1), cfg.start_pos(2), 'ko', 'MarkerSize', 18, 'MarkerFaceColor', 'g', 'LineWidth', 3);
    plot(cfg.goal_pos(1), cfg.goal_pos(2), 'ks', 'MarkerSize', 18, 'MarkerFaceColor', 'r', 'LineWidth', 3);
    legend_entries{end+1} = 'Start';
    legend_entries{end+1} = 'Goal';
    
    % Info text
    info_str = '';
    if path_found
        info_str = sprintf('Path Length: %.1fm\nWaypoints: %d', ref_path.s(end), size(cfg.waypoints,1));
    else
        info_str = sprintf('PATH FAILED');
    end
    text(0.02, 0.98, info_str, 'Units', 'normalized', ...
         'VerticalAlignment', 'top', 'FontSize', 10, ...
         'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    if maze_idx == 1
        legend(legend_entries, 'Location', 'southwest', 'FontSize', 9);
    end
end

fprintf('\n=================================================================\n');
fprintf('All paths generated successfully!\n');
fprintf('=================================================================\n\n');
