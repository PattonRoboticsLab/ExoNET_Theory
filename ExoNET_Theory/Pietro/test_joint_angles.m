function test_joint_angles(robot)
% Test script to verify computeJointAngles3D function
% This creates known configurations and tests the inverse kinematics

if nargin < 1
    error('Please provide the robot object as input');
end

% Create test configurations
test_configs = [
    0, 0, 0, 0;           % Home position
    pi/4, 0, 0, 0;        % Shoulder abduction only
    0, pi/4, 0, 0;        % Elevation only
    0, 0, pi/4, 0;        % Humeral rotation only
    0, 0, 0, pi/3;        % Elbow flexion only
    pi/6, pi/6, pi/6, pi/3; % Combined movement
];

config_names = {
    'Home Position',
    'Shoulder Abduction',
    'Humeral Elevation',
    'Humeral Rotation',
    'Elbow Flexion',
    'Combined Movement'
};

fprintf('=== TESTING computeJointAngles3D ===\n\n');

for i = 1:size(test_configs, 1)
    fprintf('--- Test %d: %s ---\n', i, config_names{i});
    
    % Set robot configuration
    config = test_configs(i,:); % Direct assignment as row vector
    
    % Get forward kinematics
    T_shoulder = getTransform(robot, config, 'base', 'shoulder_abduction');
    T_elbow = getTransform(robot, config, 'base', 'upper_arm');
    T_wrist = getTransform(robot, config, 'base', 'forearm');
    
    shoulder_pos = T_shoulder(1:3,4)';
    elbow_pos = T_elbow(1:3,4)';
    wrist_pos = T_wrist(1:3,4)';
    
    % Test inverse kinematics
    try
        q_computed = computeJointAngles3D(shoulder_pos, elbow_pos, wrist_pos);
        
        % Compare results
        q_original = test_configs(i,:);
        error_deg = rad2deg(abs(q_computed - q_original));
        
        fprintf('Original angles: [%.3f, %.3f, %.3f, %.3f] rad\n', q_original);
        fprintf('                 [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q_original));
        fprintf('Computed angles: [%.3f, %.3f, %.3f, %.3f] rad\n', q_computed);
        fprintf('                 [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q_computed));
        fprintf('Error:           [%.1f, %.1f, %.1f, %.1f] deg\n', error_deg);
        
        % Check if error is acceptable
        max_error = max(error_deg);
        if max_error < 5.0
            fprintf('✓ PASS (Max error: %.1f°)\n', max_error);
        else
            fprintf('✗ FAIL (Max error: %.1f°)\n', max_error);
        end
        
        % Test forward kinematics with computed angles
        config_test = q_computed; % Direct assignment as row vector
        
        T_wrist_test = getTransform(robot, config_test, 'base', 'forearm');
        wrist_pos_test = T_wrist_test(1:3,4)';
        
        position_error = norm(wrist_pos - wrist_pos_test);
        fprintf('Position error:  %.4f m\n', position_error);
        
    catch ME
        fprintf('✗ ERROR: %s\n', ME.message);
    end
    
    fprintf('\n');
end

% Interactive test
fprintf('=== INTERACTIVE TEST ===\n');
fprintf('Click on the robot visualization to test different poses...\n');

% Create interactive visualization
figure('Name', 'Interactive Arm Test', 'Position', [100, 100, 1200, 600]);

% Initial configuration
config = homeConfiguration(robot);
subplot(1,2,1);
show(robot, config, 'Frames', 'on');
title('Current Robot Configuration');
axis equal;
grid on;

% Create sliders for joint angles
subplot(1,2,2);
hold on;

% Simple manual test with predefined poses
test_poses = [
    0.2, 0.1, 0.0;    % Shoulder
    0.1, 0.2, -0.3;   % Elbow
    0.0, 0.3, -0.6;   % Wrist
];

for test_idx = 1:3
    fprintf('Manual test %d:\n', test_idx);
    
    % Modify pose slightly
    shoulder = [0, 0, 0];
    elbow = test_poses(test_idx, :) * 0.5;
    wrist = test_poses(test_idx, :);
    
    fprintf('Shoulder: [%.3f, %.3f, %.3f]\n', shoulder);
    fprintf('Elbow:    [%.3f, %.3f, %.3f]\n', elbow);
    fprintf('Wrist:    [%.3f, %.3f, %.3f]\n', wrist);
    
    try
        q_test = computeJointAngles3D(shoulder, elbow, wrist);
        fprintf('Computed angles: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q_test));
        
        % Visualize
        config_test = q_test; % Direct assignment as row vector
        
        subplot(1,2,1);
        show(robot, config_test, 'Frames', 'on', 'PreservePlot', false);
        title(sprintf('Test %d - Computed Configuration', test_idx));
        
        pause(3);
        
    catch ME
        fprintf('Error: %s\n', ME.message);
    end
    
    fprintf('\n');
end

end