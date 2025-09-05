% Calculates angles [yaw1, pitch, yaw2, elbow] for a 4-DoF RIGHT arm with Z-Y-Z-Y joints
% Corresponding to: [shoulder_abduction, humeral_elevation, humeral_rotation, elbow]
% All angles are in radians, centered at shoulder position
% elbow = 0 when arm is extended, increases with flexion
% RIGHT ARM CONVENTION: +Y is forward, +Z is up, +X is to the right
function q = computeJointAngles3D(shoulder, elbow, wrist)
        
    % === Segment vectors (centered at shoulder) ===
    upperArm = elbow - shoulder;
    forearm = wrist - elbow;
    
    % Get segment lengths
    L1 = norm(upperArm);  % Upper arm length
    L2 = norm(forearm);   % Forearm length
    
    if L1 < eps || L2 < eps % eps è una piccola costante per evitare divisioni per zero
        warning('Lunghezza del segmento braccio/avambraccio troppo piccola. Angoli potrebbero essere indefiniti.');
        q = [0, 0, 0, 0]; % Ritorna angoli zero o NaN a seconda della gestione desiderata
        return;
    end
    
    u = upperArm / L1;  % normalized upper arm
    v = forearm / L2;   % normalized forearm
    
    %% Calculate elbow angle 
    % Elbow angle is the angle between upperArm and forearm
    % 0 = extended, π = fully flexed
    dot_product = dot(upperArm, forearm) / (L1 * L2);
    dot_product = max(-1, min(1, dot_product));  % Clamp to valid range
    elbow_angle = acos(dot_product);  % Negative because vectors point in opposite directions at elbow
    
    % Ensure elbow angle is in valid range [0, π]
    elbow_angle = max(0, min(pi, elbow_angle));
    
    %% SHOULDER ABDUCTION (yaw1) ===
    % Angolo nel piano XY, rispetto a +Z, in senso orario
    % atan2(X, Y): Z in alto, Y in avanti → usa atan2(x, y)
    yaw1 = mod(atan2(u(1), u(2)), 2*pi);  % atan2(X,Y) gives angle from +X towards +Y.
    
    %% HUMERAL ELEVATION (pitch) - angle between +Z and u
    z_axis = [0,0,1];
    pitch = acos(dot(z_axis,u)); % Angle between vertical axis and upper arm vector
    
    %% HUMERAL ROTATION (yaw2) - rotation around humerus axis (roll)
    yaw2 = -pi/2;
    % === Output ===
    q = [yaw1, pitch, yaw2, elbow_angle];


end