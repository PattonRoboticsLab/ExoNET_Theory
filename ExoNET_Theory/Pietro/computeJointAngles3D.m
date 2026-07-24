% Calculates angles [yaw1, pitch, yaw2, elbow] for a 4-DoF RIGHT arm with Z-Y-Z-Y joints
% Corresponding to: [shoulder_abduction, humeral_elevation, humeral_rotation, elbow]
% All angles are in radians, centered at shoulder position
% elbow = 0 when arm is extended, increases with flexion
% RIGHT ARM CONVENTION: +Y is forward, +Z is up, +X is to the right
% function q = computeJointAngles3D(shoulder, elbow, wrist)
% 
%     % === Segment vectors (centered at shoulder) ===
%     upperArm = elbow - shoulder;
%     forearm = wrist - elbow;
% 
%     % Get segment lengths
%     L1 = norm(upperArm);  % Upper arm length
%     L2 = norm(forearm);   % Forearm length
% 
%     if L1 < eps || L2 < eps % eps è una piccola costante per evitare divisioni per zero
%         warning('Lunghezza del segmento braccio/avambraccio troppo piccola. Angoli potrebbero essere indefiniti.');
%         q = [0, 0, 0, 0]; % Ritorna angoli zero o NaN a seconda della gestione desiderata
%         return;
%     end
% 
%     u = upperArm / L1;  % normalized upper arm
%     v = forearm / L2;   % normalized forearm
% 
%     %% Calculate elbow angle 
%     % Elbow angle is the angle between upperArm and forearm
%     % 0 = extended, π = fully flexed
%     dot_product = dot(upperArm, forearm) / (L1 * L2);
%     dot_product = max(-1, min(1, dot_product));  % Clamp to valid range
%     elbow_angle = acos(dot_product);  % Negative because vectors point in opposite directions at elbow
% 
%     % Ensure elbow angle is in valid range [0, π]
%     elbow_angle = max(0, min(pi, elbow_angle));
% 
%     %% SHOULDER ABDUCTION (yaw1) ===
%     % Angolo nel piano XY, rispetto a +Z, in senso orario
%     % atan2(X, Y): Z in alto, Y in avanti → usa atan2(x, y)
%     yaw1 = mod(atan2(u(1), u(2)), 2*pi);  % atan2(X,Y) gives angle from +X towards +Y.
% 
%     %% HUMERAL ELEVATION (pitch) - angle between +Z and u
%     z_axis = [0,0,1];
%     pitch = acos(dot(z_axis,u)); % Angle between vertical axis and upper arm vector
% 
%     %% HUMERAL ROTATION (yaw2) - rotation around humerus axis (roll)
%     yaw2 = -pi/2;
%     % === Output ===
%     q = [yaw1, pitch, yaw2, elbow_angle];
% 
% 
% end
function q = computeJointAngles3D(shoulder, elbow, wrist)
    % Vettori segmento
    upperArm = elbow - shoulder;     % u: spalla->gomito
    forearm  = wrist  - elbow;       % v: gomito->polso

    % Lunghezze
    L1 = norm(upperArm);
    L2 = norm(forearm);
    if L1 < eps || L2 < eps
        warning('Segmento troppo corto, angoli indefiniti.');
        q = [0,0,0,0]; return;
    end

    % Versori
    u = upperArm / L1;
    v = forearm  / L2;

    %% Gomito (Y): angolo tra i segmenti (0 = esteso, pi = flesso)
    c = dot(u,v); c = max(-1,min(1,c));
    elbow_angle = acos(c);

    %% Spalla Z (yaw1) e Y (pitch) per link lungo +Z
    % Se u = Rz(yaw1)*Ry(pitch)*[0;0;1], allora:
    % pitch = atan2( sqrt(u_x^2+u_y^2), u_z ), yaw1 = atan2(u_y, u_x)
    ux = u(1); uy = u(2); uz = u(3);
    pitch = atan2( hypot(ux,uy), uz );    % in [0,pi]
    yaw1  = atan2( uy, ux );              % in (-pi,pi]

    %% Spalla Z2 (yaw2, assiale): asse gomito deve allinearsi con Y locale
    % Asse gomito (in world) dal piano (u,v):
    e = cross(u, v);  ne = norm(e);
    if ne < 1e-9
        % braccio quasi esteso/collineare: assiale indefinita → fallback
        yaw2 = 0;
    else
        e = e / ne;  % unit
        % Base locale dopo Z→Y (prima dell'assiale):
        Rz = @(a)[ cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1 ];
        Ry = @(a)[ cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a) ];
        R0 = Rz(yaw1) * Ry(pitch);

        x0 = R0*[1;0;0];
        y0 = R0*[0;1;0];
        % Ruotando attorno a z0(=u) di yaw2: y' = sin(yaw2)*x0 + cos(yaw2)*y0
        % Vogliamo y' allineata a e → risolvi yaw2:
        a = dot(e, x0);
        b = dot(e, y0);
        yaw2 = atan2( a, b );  % in (-pi,pi]
    end

    % Output ordine: [Z, Y, Z, elbowY]
    q = [yaw1, pitch, yaw2, elbow_angle];
end
