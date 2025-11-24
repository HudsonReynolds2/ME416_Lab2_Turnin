function waypoints = plan_astar_path(start_pos, goal_pos, obstacles, obsR, cfg)
    %% PLAN_ASTAR_PATH - Return hardcoded waypoints from config
    % NOTE: This function has been modified to return pre-defined waypoints
    % instead of calculating A* paths. The waypoints are stored in cfg.waypoints.
    
    % Simply return the waypoints from config
    if isfield(cfg, 'waypoints') && ~isempty(cfg.waypoints)
        waypoints = cfg.waypoints;
    else
        error('No hardcoded waypoints found in config! Check cfg.waypoints or cfg.mazes(maze_idx).waypoints');
    end
end
