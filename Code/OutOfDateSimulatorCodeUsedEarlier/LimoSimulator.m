classdef LimoSimulator < handle
    %LIMOSIMULATOR Simulates Limo robot with MoCap feedback
    %
    % Features:
    %   - Kinematic bicycle model with Ackermann steering
    %   - MoCap position/orientation feedback with noise
    %   - Realistic actuator dynamics and limits
    %   - Coordinate transform simulation
    %
    % Usage:
    %   sim = LimoSimulator();
    %   sim.reset([0, 0, deg2rad(14)]);  % Start at origin, heading 14°
    %   
    %   for t = 0:dt:10
    %       [x, y, theta] = sim.getState();
    %       v = 0.2; omega = 0.1;
    %       sim.step(v, omega, dt);
    %   end
    
    properties
        % State [x, y, theta, v] in MAZE frame
        state
        
        % Parameters
        wheelbase = 0.2;      % meters (Limo wheelbase)
        max_v = 0.5;          % m/s (max linear velocity)
        max_omega = 2.0;      % rad/s (max angular velocity)
        max_steering = deg2rad(30);  % max steering angle
        
        % Noise parameters
        mocap_pos_noise = 0.002;   % m (MoCap position noise std dev)
        mocap_ang_noise = deg2rad(0.5);  % rad (MoCap angle noise)
        
        % Coordinate transform (maze -> mocap global)
        % These simulate what OptiTrack sees
        calib
        
        % Time
        time
        
        % History for plotting
        history
    end
    
    methods
        function obj = LimoSimulator()
            % Initialize simulator
            obj.state = [0; 0; 0; 0];  % [x, y, theta, v]
            obj.time = 0;
            obj.history = struct('t', [], 'x', [], 'y', [], 'theta', [], ...
                                 'v', [], 'omega', []);
            
            % Default calibration (Transform 4: x=X, y=-Z)
            obj.calib.origin_x = -2.0;
            obj.calib.origin_z = -0.8;
            obj.calib.origin_yaw = 0.0;
            obj.calib.transform = 4;
        end
        
        function reset(obj, initial_state)
            % Reset to initial state [x, y, theta] in MAZE frame
            if nargin < 2
                initial_state = [0, 0, 0];
            end
            obj.state = [initial_state(:); 0];
            obj.time = 0;
            obj.history = struct('t', [], 'x', [], 'y', [], 'theta', [], ...
                                 'v', [], 'omega', []);
        end
        
        function setCalibration(obj, transform_num)
            % Set coordinate transform (1-8)
            obj.calib.transform = transform_num;
        end
        
        function step(obj, v_cmd, omega_cmd, dt)
            % Step simulation forward by dt seconds
            % Inputs:
            %   v_cmd: commanded linear velocity (m/s)
            %   omega_cmd: commanded angular velocity (rad/s)
            %   dt: time step (s)
            
            % Apply limits
            v_cmd = max(-obj.max_v, min(obj.max_v, v_cmd));
            omega_cmd = max(-obj.max_omega, min(obj.max_omega, omega_cmd));
            
            % Current state
            x = obj.state(1);
            y = obj.state(2);
            theta = obj.state(3);
            v_prev = obj.state(4);
            
            % Simple first-order velocity dynamics
            tau_v = 0.2;  % velocity time constant (s)
            v = v_prev + (v_cmd - v_prev) * dt / tau_v;
            
            % Kinematic bicycle model (simplified - use unicycle)
            % For differential drive robot like Limo in differential mode
            x_dot = v * cos(theta);
            y_dot = v * sin(theta);
            theta_dot = omega_cmd;
            
            % Euler integration
            x_new = x + x_dot * dt;
            y_new = y + y_dot * dt;
            theta_new = wrapToPi(theta + theta_dot * dt);
            
            % Update state
            obj.state = [x_new; y_new; theta_new; v];
            obj.time = obj.time + dt;
            
            % Record history
            obj.history.t(end+1) = obj.time;
            obj.history.x(end+1) = x_new;
            obj.history.y(end+1) = y_new;
            obj.history.theta(end+1) = theta_new;
            obj.history.v(end+1) = v;
            obj.history.omega(end+1) = omega_cmd;
        end
        
        function [x, y, theta] = getState(obj)
            % Get current state in MAZE frame (ground truth)
            x = obj.state(1);
            y = obj.state(2);
            theta = obj.state(3);
        end
        
        function pose = getMocapPose(obj, add_noise)
            % Get simulated MoCap measurement in GLOBAL frame
            % Returns structure matching actual MoCap data format
            
            if nargin < 2
                add_noise = true;
            end
            
            % Ground truth in maze frame
            x_maze = obj.state(1);
            y_maze = obj.state(2);
            theta_maze = obj.state(3);
            
            % Transform maze -> global (inverse of calibration transform)
            switch obj.calib.transform
                case 1  % x_maze = +Z, y_maze = +X
                    x_global = obj.calib.origin_x + y_maze;
                    z_global = obj.calib.origin_z + x_maze;
                case 2  % x_maze = -Z, y_maze = +X
                    x_global = obj.calib.origin_x + y_maze;
                    z_global = obj.calib.origin_z - x_maze;
                case 3  % x_maze = +X, y_maze = +Z
                    x_global = obj.calib.origin_x + x_maze;
                    z_global = obj.calib.origin_z + y_maze;
                case 4  % x_maze = +X, y_maze = -Z
                    x_global = obj.calib.origin_x + x_maze;
                    z_global = obj.calib.origin_z - y_maze;
                case 5  % x_maze = -X, y_maze = +Z
                    x_global = obj.calib.origin_x - x_maze;
                    z_global = obj.calib.origin_z + y_maze;
                case 6  % x_maze = -X, y_maze = -Z
                    x_global = obj.calib.origin_x - x_maze;
                    z_global = obj.calib.origin_z - y_maze;
                case 7  % x_maze = +Z, y_maze = -X
                    x_global = obj.calib.origin_x - y_maze;
                    z_global = obj.calib.origin_z + x_maze;
                case 8  % x_maze = -Z, y_maze = -X
                    x_global = obj.calib.origin_x - y_maze;
                    z_global = obj.calib.origin_z - x_maze;
            end
            
            % Add noise if requested
            if add_noise
                x_global = x_global + randn() * obj.mocap_pos_noise;
                z_global = z_global + randn() * obj.mocap_pos_noise;
                theta_noise = randn() * obj.mocap_ang_noise;
            else
                theta_noise = 0;
            end
            
            % Yaw in global frame
            yaw_global = wrapToPi(theta_maze + obj.calib.origin_yaw + theta_noise);
            
            % Convert to quaternion [x, y, z, w]
            % Rotation about Y-axis (standard for ground robots in MoCap)
            quat = yawToQuaternion(yaw_global);
            
            % Create pose structure matching MoCap format
            pose.pos = [x_global; 0; z_global];  % [X, Y, Z]
            pose.rot = quat;  % [x, y, z, w]
            pose.time = obj.time;
        end
        
        function plotHistory(obj, path)
            % Plot simulation results
            % Optional: path structure with reference trajectory
            
            figure('Name', 'Simulation Results', 'Position', [100, 100, 1200, 800]);
            
            % Subplot 1: XY trajectory
            subplot(2, 3, [1, 4]);
            hold on; grid on; axis equal;
            
            if nargin > 1 && ~isempty(path)
                plot(path.x, path.y, 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');
            end
            
            plot(obj.history.x, obj.history.y, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
            plot(obj.history.x(1), obj.history.y(1), 'go', 'MarkerSize', 10, ...
                 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
            plot(obj.history.x(end), obj.history.y(end), 'ro', 'MarkerSize', 10, ...
                 'MarkerFaceColor', 'r', 'DisplayName', 'End');
            
            % Draw coordinate axes
            quiver(0, 0, 0.5, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, ...
                   'DisplayName', '+X');
            quiver(0, 0, 0, 0.5, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, ...
                   'DisplayName', '+Y');
            
            xlabel('X (m)'); ylabel('Y (m)');
            title('Trajectory (Maze Frame)');
            legend('Location', 'best');
            xlim([-0.5, 3]); ylim([-1, 2]);
            
            % Subplot 2: Velocity
            subplot(2, 3, 2);
            plot(obj.history.t, obj.history.v, 'b-', 'LineWidth', 1.5);
            grid on;
            xlabel('Time (s)'); ylabel('Velocity (m/s)');
            title('Linear Velocity');
            
            % Subplot 3: Angular velocity
            subplot(2, 3, 3);
            plot(obj.history.t, obj.history.omega, 'r-', 'LineWidth', 1.5);
            grid on;
            xlabel('Time (s)'); ylabel('ω (rad/s)');
            title('Angular Velocity');
            yline(0, 'k--', 'Alpha', 0.3);
            
            % Subplot 4: Heading
            subplot(2, 3, 5);
            plot(obj.history.t, rad2deg(obj.history.theta), 'b-', 'LineWidth', 1.5);
            grid on;
            xlabel('Time (s)'); ylabel('Heading (deg)');
            title('Robot Heading');
            
            % Subplot 5: Speed
            subplot(2, 3, 6);
            speed = sqrt(diff(obj.history.x).^2 + diff(obj.history.y).^2) ./ diff(obj.history.t);
            plot(obj.history.t(2:end), speed, 'b-', 'LineWidth', 1.5);
            grid on;
            xlabel('Time (s)'); ylabel('Speed (m/s)');
            title('Actual Speed');
        end
    end
end

%% Helper functions
function quat = yawToQuaternion(yaw)
    % Convert yaw angle to quaternion [x, y, z, w]
    % Rotation about Y-axis (standard for ground robots)
    quat = [
        0;                      % x
        sin(yaw/2);            % y
        0;                      % z
        cos(yaw/2)             % w
    ];
end
