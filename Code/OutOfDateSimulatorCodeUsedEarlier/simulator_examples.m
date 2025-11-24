%% SIMULATOR EXAMPLE - Simple Usage
% This shows how to use the LimoSimulator for basic testing

clear; clc; close all;

%% Example 1: Open-loop control
fprintf('=== EXAMPLE 1: Open-Loop Circle ===\n');

sim = LimoSimulator();
sim.reset([0, 0, 0]);  % Start at origin

dt = 0.05;
v = 0.2;      % 0.2 m/s forward
omega = 0.3;  % 0.3 rad/s turn rate

for t = 0:dt:10
    sim.step(v, omega, dt);
end

[x, y, theta] = sim.getState();
fprintf('Final position: (%.3f, %.3f) @ %.1f°\n', x, y, rad2deg(theta));
fprintf('Expected: robot drove in a circle\n\n');

sim.plotHistory();

%% Example 2: Test coordinate transform
fprintf('=== EXAMPLE 2: Coordinate Transform ===\n');

sim2 = LimoSimulator();
sim2.setCalibration(4);  % Use Transform 4

% Reset to origin
sim2.reset([0, 0, deg2rad(14)]);

% Get MoCap pose
pose = sim2.getMocapPose(false);  % No noise
fprintf('Robot at maze (0, 0):\n');
fprintf('  MoCap global: X=%.3f, Z=%.3f\n', pose.pos(1), pose.pos(3));
fprintf('  Quaternion: [%.3f, %.3f, %.3f, %.3f]\n', ...
        pose.rot(1), pose.rot(2), pose.rot(3), pose.rot(4));

% Drive forward
for t = 0:dt:2.5
    sim2.step(0.2, 0, dt);
end

[x_maze, y_maze, ~] = sim2.getState();
pose2 = sim2.getMocapPose(false);

fprintf('\nAfter driving forward 0.5m in +X maze direction:\n');
fprintf('  Maze frame: X=%.3f, Y=%.3f\n', x_maze, y_maze);
fprintf('  MoCap global: X=%.3f, Z=%.3f\n', pose2.pos(1), pose2.pos(3));
fprintf('  Expected: maze X increases, Y stays ~0\n\n');

%% Example 3: Simple waypoint controller
fprintf('=== EXAMPLE 3: Simple Waypoint Following ===\n');

sim3 = LimoSimulator();
sim3.reset([0, 0, deg2rad(14)]);

waypoint = [0.8, 0.2];
K_p = 2.0;  % Proportional gain for heading

for t = 0:dt:15
    [x, y, theta] = sim3.getState();
    
    % Compute heading to waypoint
    dx = waypoint(1) - x;
    dy = waypoint(2) - y;
    desired_heading = atan2(dy, dx);
    
    % Heading error
    heading_error = wrapToPi(desired_heading - theta);
    
    % Simple proportional control
    omega = K_p * heading_error;
    omega = max(-1.0, min(1.0, omega));  % Saturate
    
    v = 0.2;
    sim3.step(v, omega, dt);
    
    % Stop when close
    dist = hypot(dx, dy);
    if dist < 0.05
        fprintf('Reached waypoint at t=%.1fs\n', t);
        break;
    end
end

[x_final, y_final, ~] = sim3.getState();
fprintf('Final position: (%.3f, %.3f)\n', x_final, y_final);
fprintf('Target was: (%.3f, %.3f)\n', waypoint(1), waypoint(2));
fprintf('Error: %.3f m\n\n', hypot(x_final - waypoint(1), y_final - waypoint(2)));

sim3.plotHistory();

%% Example 4: Test with MoCap noise
fprintf('=== EXAMPLE 4: MoCap Noise Test ===\n');

sim4 = LimoSimulator();
sim4.reset([0, 0, 0]);

% Drive straight
for t = 0:dt:5
    sim4.step(0.2, 0, dt);
end

% Get multiple MoCap readings with noise
fprintf('Taking 10 MoCap readings with noise:\n');
for i = 1:10
    pose = sim4.getMocapPose(true);  % With noise
    fprintf('  Reading %d: X=%.4f, Z=%.4f\n', i, pose.pos(1), pose.pos(3));
end

[x_true, y_true, ~] = sim4.getState();
fprintf('Ground truth: (%.4f, %.4f)\n', x_true, y_true);
fprintf('Notice: small variations due to simulated MoCap noise\n');

fprintf('\n=== All examples complete! ===\n');
