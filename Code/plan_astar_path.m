function waypoints = plan_astar_path(start_pos, goal_pos, obstacles, obsR, cfg)
    %% PLAN_ASTAR_PATH - Return hardcoded waypoints from config
    %
    % A* is an informed search algorithm that uses a heuristic h(n) to guide
    % the search toward the goal. It expands nodes with minimum f(n) = g(n) + h(n)
    % where g(n) is cost from start and h(n) is estimated cost to goal.
    %
    % A* is complete and optimal when h(n) is admissible (never overestimates).
    % Common heuristics: Euclidean distance, Manhattan distance.
    %
    % NOTE: This version returns pre-defined waypoints instead of computing A*.

    if isfield(cfg, 'waypoints') && ~isempty(cfg.waypoints)
        waypoints = cfg.waypoints;
    else
        error('No hardcoded waypoints found in config!');
    end
end
