clear; clc; close all;

%% ======================= OPTITRACK QUATERNION TEST =======================
% Determines correct quaternion-to-Euler conversion for OptiTrack Y-up system
% 
% INSTRUCTIONS:
% 1. Place robot at origin facing "forward" (note which direction)
% 2. Run script and follow on-screen prompts
% 3. Script will determine which formula gives correct yaw
%
% OptiTrack Info:
% - Y-up coordinate system
% - Quaternion format in MQTT: [qx, qy, qz, qw] where qw is real part
% ==========================================================================

fprintf('=================================================================\n');
fprintf('OPTITRACK QUATERNION EXTRACTION TEST\n');
fprintf('Y-up Coordinate System\n');
fprintf('=================================================================\n\n');

%% MQTT Setup
broker = "mqtt://rasticvm.lan";
mq = mqttclient(broker, Port=1883, ClientID="quat_tester");
if ~mq.Connected, error("Could not connect to MQTT"); end

topic_pose = "rb/limo809";  % Change if needed
topic_cmd = "rb/limo809/cmd";
subscribe(mq, topic_pose);

fprintf('Connected to MQTT: %s\n\n', broker);

%% Stop robot first
fprintf('Stopping robot...\n');
for i = 1:10
    write(mq, topic_cmd, jsonencode(struct("v", 0, "w", 0)));
    pause(0.05);
end

%% Test 1: Initial orientation
fprintf('=== TEST 1: INITIAL ORIENTATION ===\n');
fprintf('1. Place robot at origin\n');
fprintf('2. Point robot in your "forward" direction (typically +X in maze)\n');
fprintf('3. Press ENTER when ready...\n');
pause;

initial = getAngles(mq, topic_pose);
displayAllAngles('Initial Position', initial);

fprintf('\nWhich direction is the robot facing in the ROOM?\n');
fprintf('(e.g., toward door, toward window, north, etc.): ');
room_direction = input('', 's');

%% Test 2: 90-degree rotation
fprintf('\n=== TEST 2: 90-DEGREE ROTATION ===\n');
fprintf('Manually rotate robot 90 degrees COUNTER-CLOCKWISE (turn left)\n');
fprintf('Press ENTER when done...\n');
pause;

rot90 = getAngles(mq, topic_pose);
displayAllAngles('After 90° CCW', rot90);

% Calculate differences
fprintf('\n--- Angle Changes from Initial ---\n');
changes = struct();
fields = fieldnames(initial);
for i = 1:length(fields)
    f = fields{i};
    if ~strcmp(f, 'pos') && ~strcmp(f, 'quat')
        changes.(f) = wrapToPi(rot90.(f) - initial.(f));
        fprintf('%-15s: %+7.1f°\n', f, rad2deg(changes.(f)));
    end
end

%% Test 3: 180-degree rotation
fprintf('\n=== TEST 3: 180-DEGREE ROTATION ===\n');
fprintf('Rotate robot to face BACKWARD (180° from initial)\n');
fprintf('Press ENTER when done...\n');
pause;

rot180 = getAngles(mq, topic_pose);
displayAllAngles('After 180°', rot180);

fprintf('\n--- Angle Changes from Initial ---\n');
changes180 = struct();
for i = 1:length(fields)
    f = fields{i};
    if ~strcmp(f, 'pos') && ~strcmp(f, 'quat')
        changes180.(f) = wrapToPi(rot180.(f) - initial.(f));
        fprintf('%-15s: %+7.1f°\n', f, rad2deg(changes180.(f)));
    end
end

%% Test 4: -90-degree rotation
fprintf('\n=== TEST 4: -90-DEGREE ROTATION ===\n');
fprintf('Rotate robot 90 degrees CLOCKWISE from initial (turn right)\n');
fprintf('Press ENTER when done...\n');
pause;

rotNeg90 = getAngles(mq, topic_pose);
displayAllAngles('After 90° CW', rotNeg90);

fprintf('\n--- Angle Changes from Initial ---\n');
changesNeg90 = struct();
for i = 1:length(fields)
    f = fields{i};
    if ~strcmp(f, 'pos') && ~strcmp(f, 'quat')
        changesNeg90.(f) = wrapToPi(rotNeg90.(f) - initial.(f));
        fprintf('%-15s: %+7.1f°\n', f, rad2deg(changesNeg90.(f)));
    end
end

%% Analysis
fprintf('\n=================================================================\n');
fprintf('ANALYSIS\n');
fprintf('=================================================================\n\n');

fprintf('For a Y-up coordinate system with robot rotating on ground plane:\n');
fprintf('- The YAW angle should change when robot turns\n');
fprintf('- PITCH and ROLL should stay near zero\n\n');

% Find which angle changed most consistently
analyze_rotation(changes, 90, 'After 90° CCW');
analyze_rotation(changes180, 180, 'After 180°');
analyze_rotation(changesNeg90, -90, 'After 90° CW');

%% Recommendation
fprintf('\n=================================================================\n');
fprintf('RECOMMENDATION\n');
fprintf('=================================================================\n\n');

% Determine which extraction method worked best
scores = evaluate_methods(changes, changes180, changesNeg90);
[~, best_idx] = max(scores);

method_names = {'yaw_zyx', 'yaw_yup', 'yaw_alt', 'roll_zyx', 'pitch_zyx', 'yaw_custom'};
fprintf('Best extraction method: %s\n\n', method_names{best_idx});

fprintf('Use this in your code:\n');
fprintf('----------------------------------------\n');
switch best_idx
    case 1
        fprintf('// Standard ZYX Yaw\n');
        fprintf('yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));\n');
    case 2
        fprintf('// Y-up Yaw (most likely for OptiTrack)\n');
        fprintf('yaw = atan2(2*(qw*qy + qz*qx), 1 - 2*(qx^2 + qz^2));\n');
    case 3
        fprintf('// Alternative Yaw\n');
        fprintf('yaw = atan2(2*(qw*qy - qz*qx), 1 - 2*(qx^2 + qy^2));\n');
    case 4
        fprintf('// Using Roll as Yaw\n');
        fprintf('yaw = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));\n');
    case 5
        fprintf('// Using Pitch as Yaw\n');
        fprintf('yaw = asin(clamp(2*(qw*qy - qz*qx), -1, 1));\n');
    case 6
        fprintf('// Your current (probably wrong)\n');
        fprintf('yaw = atan2(2*(qw*qy + qx*qz), 1 - 2*(qy^2 + qz^2));\n');
end
fprintf('----------------------------------------\n');

%% Continuous monitoring mode
fprintf('\n=== CONTINUOUS MONITORING ===\n');
fprintf('Move robot around to verify angles\n');
fprintf('Press Ctrl+C to exit\n\n');

fig = figure('Name', 'OptiTrack Angles', 'Position', [100 100 800 600]);
while true
    try
        current = getAngles(mq, topic_pose);
        
        clf;
        subplot(2,1,1);
        bar([rad2deg(current.yaw_yup), rad2deg(current.yaw_zyx), ...
             rad2deg(current.roll_zyx), rad2deg(current.pitch_zyx)]);
        xticklabels({'Yaw (Y-up)', 'Yaw (ZYX)', 'Roll', 'Pitch'});
        ylabel('Degrees');
        title('Current Angles');
        grid on;
        ylim([-180 180]);
        
        subplot(2,1,2);
        compass(cos(current.yaw_yup), sin(current.yaw_yup));
        title(sprintf('Robot Heading (Y-up Yaw): %.1f°', rad2deg(current.yaw_yup)));
        
        drawnow;
        pause(0.1);
    catch ME
        if contains(ME.identifier, 'interrupt')
            break;
        end
    end
end

delete(mq);
close all;
fprintf('\nTest complete!\n');

%% ======================= HELPER FUNCTIONS =======================

function angles = getAngles(mq, topic)
    % Get all possible angle extractions from quaternion
    
    pause(0.1);
    tbl = [];
    attempts = 0;
    while isempty(tbl) && attempts < 50
        tbl = read(mq, Topic=topic);
        pause(0.02);
        attempts = attempts + 1;
    end
    
    if isempty(tbl)
        error('No data received from MQTT');
    end
    
    data = jsondecode(tbl.Data{end});
    
    % Store position and quaternion
    angles.pos = data.pos;
    angles.quat = data.rot;  % [qx, qy, qz, qw]
    
    % Extract quaternion components
    qx = data.rot(1);
    qy = data.rot(2);
    qz = data.rot(3);
    qw = data.rot(4);  % Real part
    
    % Method 1: Standard ZYX Euler angles (aerospace convention)
    angles.roll_zyx = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));
    angles.pitch_zyx = asin(clamp(2*(qw*qy - qz*qx), -1, 1));
    angles.yaw_zyx = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
    
    % Method 2: Y-up convention (common for OptiTrack/graphics)
    % In Y-up, rotation around Y is yaw
    angles.yaw_yup = atan2(2*(qw*qy + qz*qx), 1 - 2*(qx^2 + qz^2));
    
    % Method 3: Alternative extraction
    angles.yaw_alt = atan2(2*(qw*qy - qz*qx), 1 - 2*(qx^2 + qy^2));
    
    % Method 4: Your current formula (likely wrong)
    angles.yaw_custom = atan2(2*(qw*qy + qx*qz), 1 - 2*(qy^2 + qz^2));
end

function displayAllAngles(label, angles)
    fprintf('\n%s:\n', label);
    fprintf('  Position: [%.3f, %.3f, %.3f]\n', angles.pos(1), angles.pos(2), angles.pos(3));
    fprintf('  Quaternion: [%.3f, %.3f, %.3f, %.3f]\n', ...
            angles.quat(1), angles.quat(2), angles.quat(3), angles.quat(4));
    fprintf('  --- Extracted Angles ---\n');
    fprintf('  Roll (ZYX):    %7.1f°\n', rad2deg(angles.roll_zyx));
    fprintf('  Pitch (ZYX):   %7.1f°\n', rad2deg(angles.pitch_zyx));
    fprintf('  Yaw (ZYX):     %7.1f°\n', rad2deg(angles.yaw_zyx));
    fprintf('  Yaw (Y-up):    %7.1f°\n', rad2deg(angles.yaw_yup));
    fprintf('  Yaw (Alt):     %7.1f°\n', rad2deg(angles.yaw_alt));
    fprintf('  Yaw (Current): %7.1f°\n', rad2deg(angles.yaw_custom));
end

function analyze_rotation(changes, expected_deg, label)
    fprintf('\n%s (expected %.0f° change):\n', label, expected_deg);
    
    expected_rad = deg2rad(expected_deg);
    errors = struct();
    
    % Check each angle
    errors.yaw_zyx = abs(wrapToPi(changes.yaw_zyx - expected_rad));
    errors.yaw_yup = abs(wrapToPi(changes.yaw_yup - expected_rad));
    errors.yaw_alt = abs(wrapToPi(changes.yaw_alt - expected_rad));
    errors.roll = abs(changes.roll_zyx);  % Should be ~0
    errors.pitch = abs(changes.pitch_zyx);  % Should be ~0
    errors.yaw_custom = abs(wrapToPi(changes.yaw_custom - expected_rad));
    
    % Find best match
    [min_err, best] = min([errors.yaw_zyx, errors.yaw_yup, errors.yaw_alt, ...
                           errors.roll, errors.pitch, errors.yaw_custom]);
    
    methods = {'yaw_zyx', 'yaw_yup', 'yaw_alt', 'roll_zyx', 'pitch_zyx', 'yaw_custom'};
    
    fprintf('  Best match: %s (error: %.1f°)\n', methods{best}, rad2deg(min_err));
    
    if min_err < deg2rad(10)
        fprintf('  ✓ GOOD MATCH\n');
    else
        fprintf('  ✗ Poor match - check rotation direction\n');
    end
end

function scores = evaluate_methods(changes90, changes180, changesNeg90)
    % Score each method based on how well it matches expected rotations
    
    scores = zeros(1, 6);
    
    % Expected values (in radians)
    exp90 = deg2rad(90);
    exp180 = deg2rad(180);
    expNeg90 = deg2rad(-90);
    
    % Check each method
    methods = {'yaw_zyx', 'yaw_yup', 'yaw_alt', 'roll_zyx', 'pitch_zyx', 'yaw_custom'};
    
    for i = 1:length(methods)
        m = methods{i};
        
        % Calculate errors
        err90 = abs(wrapToPi(changes90.(m) - exp90));
        err180 = abs(wrapToPi(changes180.(m) - exp180));
        errNeg90 = abs(wrapToPi(changesNeg90.(m) - expNeg90));
        
        % Score inversely proportional to error
        total_err = err90 + err180 + errNeg90;
        scores(i) = 1 / (1 + total_err);
    end
end

function val = clamp(val, minval, maxval)
    val = max(min(val, maxval), minval);
end
