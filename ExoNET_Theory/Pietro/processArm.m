function [velocity, acceleration, torque] = processArm(Body, time, Bod, armSide, filename)
% PROCESSARM - Compute arm dynamics (velocity, acceleration, torques)
% Inputs:
%   Body: struct with joint positions
%   time: time vector (unused)
%   Bod: biomechanical parameters (unused here)
%   robot: rigidBodyTree model
%   armSide: 'l' or 'r'

%% --- Initialization ---
velocity = struct();
acceleration = struct();
torque = struct();

% Determine side
if lower(armSide) == 'l'
    armLabel = 'Left';
else
    armLabel = 'Right';
end

% Extract positions
shoulder = Body.Shoulder;
elbow    = Body.Elbow;
wrist    = Body.Wrist;

% Clean invalid data
validRows = ~any(isnan([shoulder, elbow, wrist]), 2);
shoulder = shoulder(validRows,:);
elbow    = elbow(validRows,:);
wrist    = wrist(validRows,:);

%% Parameters for sgolay and for the interpolation
fs = 30;        dt = 1/fs;

%% Blow up useless points
shoulder_filt = shoulder( 90:end-90, :);
elbow_filt    = elbow( 90:end-90, :);
wrist_filt    = wrist( 90:end-90, :);

time_pos = ( 0:length( elbow_filt ) - 1 )' * dt;

%% Design 10 Hz Low-Pass Butterworth filter
[b, a] = butter( 7, 3 / ( fs / 2 ) ); 

%% --- Filtering positions with filtfilt ---
shoulder_filt = filtfilt( b, a, shoulder_filt );
elbow_filt    = filtfilt( b, a, elbow_filt );
wrist_filt    = filtfilt( b, a, wrist_filt );

shoulder_filt = movmean( shoulder_filt, 13, 'Endpoints', 'shrink');
elbow_filt    = movmean( elbow_filt,    13, 'Endpoints', 'shrink');
wrist_filt    = movmean( wrist_filt,    13, 'Endpoints', 'shrink');

%% Create robot model
upperarm_length = mean( vecnorm( elbow_filt - shoulder_filt, 2, 2 ) );
forearm_length  = mean( vecnorm( wrist_filt - elbow_filt,    2, 2 ) );
robot = create3DArmModel( upperarm_length, forearm_length, Bod ); 
%showdetails(robot);

%% --- Velocities and Accelerations ---
vel_upper = diff(elbow_filt) / dt;
vel_upper_filt   = filtfilt( b, a, vel_upper );

acc_upper = diff(vel_upper_filt) / dt;
acc_upper_filt   = filtfilt( b, a, acc_upper );

vel_fore = diff(wrist_filt) / dt;
vel_fore_filt = filtfilt( b, a, vel_fore );

acc_fore = diff(vel_fore_filt) / dt;
acc_fore_filt = filtfilt( b, a, acc_fore );
  
% Before evaluating blow up points depending on how is the signal (see it)
% vel_upper_filt   = vel_upper_filt(90:end-90,:);
% acc_upper_filt   = acc_upper_filt(90:end-92,:);
% vel_forearm_filt = vel_forearm_filt(90:end-90,:);
% acc_forearm_filt = acc_forearm_filt(90:end-92,:);

%% Magnitudes
vel_mag_upper   = vecnorm( vel_upper_filt,  2, 2 );
acc_mag_upper   = vecnorm( acc_upper_filt,  2, 2 );
vel_mag_fore    = vecnorm( vel_fore_filt,   2, 2 );
acc_mag_fore    = vecnorm( acc_fore_filt,   2, 2 );

%% Store velocities and acceleratio
velocity.upperarm_vector = vel_upper_filt;
velocity.forearm_vector  = vel_fore_filt;
velocity.upperarm        = vel_mag_upper;
velocity.forearm         = vel_mag_fore;

acceleration.upperarm_vector = acc_upper_filt;
acceleration.forearm_vector  = acc_fore_filt;
acceleration.upperarm        = acc_mag_upper;
acceleration.forearm         = acc_mag_fore;

%% --- Joint angles and derivatives ---
nSamples = size(shoulder_filt,1);
q = zeros(nSamples,4);

% shoulder = [0 0 0];
% elbow    = [1 0 0];     % upper arm: along X
% wrist    = [1 1 0];     % forearm: bent in Y direction
% 
% q = computeJointAnglesRobust(shoulder, elbow, wrist);
% disp(rad2deg(q))  % show degrees

for i = 1:nSamples
    q(i,:) = computeJointAnglesRobust(shoulder_filt(i,:), elbow_filt(i,:), wrist_filt(i,:));
end

q = unwrap(q, deg2rad(320)); 

q_filt = filtfilt( b, a, q );
q_filt = movmean( q_filt, 13, 'Endpoints', 'shrink' );
q = q_filt;   

%% --- Inverse dynamics to evaluate gravity torque ---
nSamples_dyn = size(q,1);
torques_gravity  = zeros(nSamples_dyn, 4);

for i = 1:nSamples_dyn
    torques_gravity(i,:)  = inverseDynamics( robot, q(i,:) ); % Find the gravity torque
end

%Filtering torques
torques_gravity  = filtfilt(b, a, torques_gravity);

shoulder_gravity  = vecnorm( torques_gravity,  2, 2 );

torque.shoulder_gravity = shoulder_gravity;     
torque.elbow_gravity = torques_gravity(:,4);

%% --- Time vectors ---
t_vel_original     = (0:length(velocity.upperarm)-1)' * dt;
t_acc_original     = (0:length(acceleration.upperarm)-1)' * dt;
t_torque_original  = (0:length(torque.shoulder_gravity)-1)' * dt;

% Update time vectors
time_velocity     = t_vel_original;
time_acceleration = t_acc_original;
time_torque       = t_torque_original;

%% --- Plotting ---
figure('Name', ['Upper Arm Gravity - ' armLabel ' Arm']);
subplot(5,1,1); plot(time_pos,          vecnorm(elbow_filt-shoulder_filt, 2, 2), 'g'); ylabel('m'); title('Upperarm length');
subplot(5,1,2); plot(time_velocity,     velocity.upperarm, 'r');                       ylabel('Velocity (m/s)'); title('Upper Arm Velocity');
subplot(5,1,3); plot(time_acceleration, acceleration.upperarm, 'g');                   ylabel('Acceleration (m/s²)'); title('Upper Arm Acceleration');
subplot(5,1,4); plot(time_torque,       torque.shoulder_gravity, 'k');                 ylabel('Torque (Nm)'); xlabel('Time (s)'); title('Shoulder Torque');
subplot(5,1,5); plot(time_torque,       rad2deg(q(:,1)), 'r'); hold on; 
                plot(time_torque,       rad2deg(q(:,2)), 'g' ); title('q1 and q2');

figure('Name', ['Forearm Gravity - ' armLabel ' Arm']);
subplot(5,1,1); plot(time_pos,          vecnorm(wrist_filt-elbow_filt, 2, 2), 'r'); ylabel('m'); title('Forearm length');
subplot(5,1,2); plot(time_velocity,     velocity.forearm, 'r');                     ylabel('Velocity (m/s)'); title('Forearm Velocity');
subplot(5,1,3); plot(time_acceleration, acceleration.forearm, 'g');                 ylabel('Acceleration (m/s²)'); title('Forearm Acceleration');
subplot(5,1,4); plot(time_torque,       torque.elbow_gravity, 'k');                 ylabel('Torque (Nm)'); xlabel('Time (s)'); title('Elbow Torque');
subplot(5,1,5); plot(time_torque,       rad2deg(q(:,4)), 'r'); hold on; 
                plot(time_torque,       rad2deg(q(:,3)), 'g' ); title('q3 and q4');

%% --- Summary on Command Window ---
fprintf('\nSummary for the %s arm\n', armLabel);
fprintf('Average Shoulder Gravity Torque: %.2f Nm\n',  mean(torque.shoulder_gravity));
fprintf('Average Elbow Gravity Torque:    %.2f Nm\n',  mean(torque.elbow_gravity));

fprintf('Average Upper Arm Velocity:      %.2f m/s\n',    mean(velocity.upperarm));
fprintf('Average Forearm Velocity:        %.2f m/s\n',    mean(velocity.forearm));
fprintf('Average Upper Arm Acceleration:  %.2f m/s²\n',   mean(acceleration.upperarm));
fprintf('Average Forearm Acceleration:    %.2f m/s²\n\n', mean(acceleration.forearm));

% figure('Name','torque gravity')
% histogram(shoulder_gravity)
% 
% figure('Name','upper acc')
% histogram(acc_upper_filt)
% 
% figure('Name','upper vel')
% histogram(vel_upper_filt)
% 
% figure('Name','forearm acc')
% histogram(acc_forearm_filt)
% 
% figure('Name','forearm vel')
% histogram(vel_forearm_filt)

end
