% Function that evaluate the positions of the body and of the CenterMass(CM) in 3D
function [Pos, RotMatrix, Jacobian_ShEl, Jacobian_swivel] = forwardKin3D(PHIs, Bod) 
L1 = Bod.L(1);  L2 = Bod.L(2);  % Length of the upper arm and forearm
R1 = Bod.R(1);  R2 = Bod.R(2);  % Proximal to centers of mass of segments in percentage

%% Evaluate all the points of the arm in different planes for ELEVATION
Pos.sh    = zeros(size(PHIs.el));         % zeros for this one shoulder positons, position evaluated from the shoulder
phi12     = PHIs.el(:,1) + PHIs.el(:,2);  % sum phi's 

% Position of the upper arm and its center of mass
Pos.el    = L1 * [ cos(PHIs.el(:,3)).*cos(PHIs.el(:,1)), sin(PHIs.el(:,3)).*cos(PHIs.el(:,1)), sin(PHIs.el(:,1)) ];
Pos.R1    = R1 .* Pos.el ;              

% Position of the forearm and its center of mass
Pos.el2wr = L2 * [ cos(PHIs.el(:,3)).*cos(phi12), sin(PHIs.el(:,3)).*cos(phi12), sin(phi12) ] ;        
Pos.el2R2 = R2 .* Pos.el2wr;    % elbow2forearm CM in 3D

Pos.R2    = Pos.el + Pos.el2R2; % Position of the forearm
Pos.wr    = Pos.el + Pos.el2wr; % Positon of the wrist

% Position of the gap where the bands are attached 
Pos.swgap = Pos.el + Bod.gap * [ cos( PHIs.el(:,3) - ones( length(PHIs.el),1 ) * pi/2 ),...
                                 sin( PHIs.el(:,3) - ones( length(PHIs.el),1 ) * pi/2),...
                                 zeros( length(PHIs.el),1 ) ];

Pos.elgap = Pos.el + Bod.gap * [ cos( PHIs.el(:,3)) .* cos( PHIs.el(:,1) + ones( length(PHIs.el),1 ) * pi/2 ), ...
                                 sin( PHIs.el(:,3)) .* cos( PHIs.el(:,1) + ones( length(PHIs.el),1 ) * pi/2 ),...
                                 sin( PHIs.el(:,1) + ones( length(PHIs.el),1 )*pi/2) ];

%% Position for SWIVEL used to have the same amount of position of both movements
phi12sw      = PHIs.sw(:,1) + PHIs.sw(:,2);

% Position of the upper arm and its center of mass swivel
Pos.elsw     = L1 .* [ cos(PHIs.sw(:,3)).*cos(PHIs.sw(:,1)), sin(PHIs.sw(:,3)).*cos(PHIs.sw(:,1)), sin(PHIs.sw(:,1)) ];    
Pos.R1sw     = R1 .* Pos.elsw;

% Position of the forearm and its center of mass swivel
Pos.el2wrsw  = L2 * [ cos(PHIs.sw(:,3)).*cos(phi12sw), sin(PHIs.sw(:,3)).*cos(phi12sw), sin(phi12sw) ];   
Pos.Rel2wrsw = R2 * Pos.el2wrsw;

Pos.R2sw    = Pos.elsw + Pos.Rel2wrsw; % Position of the forearm swivel
Pos.wrsw    = Pos.elsw + Pos.el2wrsw; % Positon of the wrist

Pos.elgapsw = Pos.elsw + Bod.gap * [cos( PHIs.sw(:,3)) .* cos( PHIs.sw(:,1) + ones( length(PHIs.sw),1 ) * pi/2) , ...
                                    sin( PHIs.sw(:,3)) .* cos( PHIs.sw(:,1) + ones( length(PHIs.sw),1 ) * pi/2) ,...
                                    sin( PHIs.sw(:,1) + ones( length(PHIs.sw),1 ) * pi/2)];

%% Evaluate the positions of the elbow as humeral elevation using swivel angle
% Define the range of rotation (e.g., 90° rightward from the starting 
% point) since it is a right arm. In case for the left arm it should work
% symmetric
rotation_angle = pi / 2;  % 90 degrees
theta = linspace(pi/2, rotation_angle+pi/2, Bod.n_angles_3D);  % Limited swivel angle 
RotMatrix = []; % Initialize RotMatrix

for i = 1:length(Pos.wrsw)    
    % Create axis unit for RotMatrix  
    % axis_unit = Pos.wrsw(i, :)-Pos.elsw(i,:) / norm(Pos.wrsw(i, :)-Pos.elsw(i,:)); % Normalize the shoulder-wrist axis
    axis_unit = Pos.wrsw(i, :) / norm(Pos.wrsw(i, :)); % Normalize the shoulder-wrist axis 

    % angle is the angle of rotation of the shoulder that appears in zy
    % plane and that, after the rotation of the elbow, appears in xy plane
    elevation_wr = acos(max(min(Pos.wrsw(i,3) / norm(Pos.wrsw(i, :)), 1), -1));
    % 
    angle_shoulder = acos(max(min(Pos.elsw(i,3) / norm(Pos.elsw(i, :)), 1), -1)) - elevation_wr;    
    angle_gap_elevation = acos(max(min(Pos.elgapsw(i,3) / norm(Pos.elgapsw(i, :)), 1), -1)) - elevation_wr;

    % Elbow's perpendicular distance from the shoulder-wrist axis,
    % evaluated, for the first position of the elbow for every position
    Pos.center_axis(i,:)    = ( L1 * cos( angle_shoulder )) * axis_unit;
    Pos.center_axis_el(i,:) = ( norm(Pos.elgapsw(i,:)) * cos( angle_gap_elevation )) * axis_unit;
    % 
    % elbow_radius = norm(Pos.center_axis(i,:) - Pos.elsw(i,:));       % Radius of the circular path
    % gapel_radius = norm(Pos.center_axis_el(i,:) - Pos.elgapsw(i,:)); % Radius of the circular path for elevation gap

    elbow_axis   = -Pos.elsw(i,:) / norm(Pos.elsw(i,:));
    
    % Generate perp unit vector for the plane
    perp_vector = cross(axis_unit, elbow_axis);  % First perpendicular vector
    perp_unit = perp_vector / norm(perp_vector); % Find the perp. unit respect the axis wrist-sholder

    % Generate circle unit for rotation
    %circle_unit = cross(axis_unit, perp_unit);   % Second perpendicular vector, to draw the part of the circle
    circle_unit = cross(perp_unit, axis_unit);   % Second perpendicular vector, to draw the part of the circle

    % Store the vectors into the rotation matrix
    RotMatrix = [ RotMatrix, axis_unit', perp_unit', circle_unit' ];

    % Evaluate all elbow positions for the rotation of the elbow
    for j = 1:length(theta)
        % Elbow position for the current theta
        % Pos.el_3D(i, j, :) = elbow_radius * (cos(theta(j)) * circle_unit +...
        %     sin(theta(j)) * perp_unit) + (L1 * cos(angle_shoulder)) * axis_unit; % Projection along the shoulder-wrist axis 
        % 
        % gap_swivel_addition = reshape(Bod.gap * ( cos(theta(j) + pi/2) * circle_unit +...
        %                                   sin(theta(j) + pi/2) * perp_unit + 0 ), [1, 1, 3]);
        % Pos.gap_swivelsw(i,j,:) = Pos.el_3D(i, j, :) + gap_swivel_addition;
        % 
        % Pos.gap_swivelel(i,j,:) = gapel_radius * (cos(theta(j)) * circle_unit +...
        %     sin(theta(j)) * perp_unit) + ( norm( Pos.elgapsw(i,:) ) * cos(angle_gap_elevation) ) * axis_unit;    
        Pos.wr_3D(i,j,:) = Pos.elsw(i,:) + L2 * (cos(theta(j)) * perp_unit + sin(theta(j)) * axis_unit) ; % Projection along the shoulder-wrist axis 
    end  
end

%% Find the position of the elbow and of the gap of the swivel angle
tau_numb = 1; % Initialize the number of torque forces for the vector
for k = 1:size(Pos.wr_3D, 1) % For each position of the elbow
    % elbow_positions = squeeze(Pos.el_3D(k, :, :));  % Extract elbow positions for the current wrist
    % gap_swivel = squeeze(Pos.gap_swivelsw(k, :, :));
    % gap_elevation = squeeze(Pos.gap_swivelel(k, :, :));
    wrist_position = squeeze(Pos.wr_3D(k,:,:));
    for j = 1:size(Pos.wr_3D, 2) % For each position of the elbow we have the rotation = n_angles_3D
        % Pos.elSwivel(tau_numb,:)    = elbow_positions(j,:); 
        % Pos.gapelSwivel(tau_numb,:) = gap_elevation(j,:);
        % Pos.gapswSwivel(tau_numb,:) = gap_swivel(j,:);
        % Pos.wristSwivel(tau_numb,:) = Pos.wrsw(k,:);
        Pos.elbowSwivel(tau_numb,:) = Pos.elsw(k,:);
        Pos.wrSwivel(tau_numb,:) = wrist_position(j,:);
        tau_numb = tau_numb + 1;
    end
end

%% Plot animation of the arm in the two solutions
%plotanimation(Pos.wr, Pos.el);
plotanimation(Pos.wrSwivel, Pos.elbowSwivel);

%% Create the matrix of the Jacobian
Jacobian_ShEl = [];
syms phi1 phi2 L_upper L_fore
J = [L_upper, L_fore] * [ cos(phi1),              sin(phi1);
                 cos(phi1 + phi2),   sin(phi1 + phi2) ];
%J = gpuArray(J);
Jacobians = jacobian(J, [phi1, phi2]);

for i = 1:size(PHIs.el, 1)  
    Jacobians_sub = subs(Jacobians, {phi1, phi2, L_upper, L_fore}, {PHIs.el(i, 1), PHIs.el(i, 2), L1, L2});
    Jacobian_ShEl = [Jacobian_ShEl, Jacobians_sub];
end
Jacobian_ShEl = double(Jacobian_ShEl);
%Jacobian_ShEL = gpuArray(double(Jacobian_ShEl));

%% Create the Jacobian for the swivel angle
Jacobian_swivel = [];
syms elr sw_angle 
Jacob = elr * [ cos(sw_angle) , sin(sw_angle), 0 ];  % Rotation in the plane
%Jacob = gpuArray (Jacob);
Jacobians_sw = jacobian( Jacob, sw_angle );
Jacobian_swivel = [];
for j = 1:length(theta) 
    Jacobians_SwSh = subs(Jacobians_sw, {elr, sw_angle}, {elbow_radius, theta(j)});
    Jacobian_swivel = [Jacobian_swivel, Jacobians_SwSh];
end
Jacobian_swivel = double(Jacobian_swivel);
%Jacobian_swivel = gpuArray(double(Jacobian_swivel));

end % End of the function

