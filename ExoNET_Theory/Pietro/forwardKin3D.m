% Function that evaluate the positions of the body and of the CenterMass(CM) in 3D
function [Pos, RotMatrix, Jacobian_ShEl] = forwardKin3D(PHIs, Bod, Exo, Pos, mode) 

L1 = Bod.L(1);  L2 = Bod.L(2);  % Length of the upper arm and forearm
R1 = Bod.R(1);  R2 = Bod.R(2);  % Proximal to centers of mass of segments in percentage

%% Evaluate all the points of the arm in different planes for ELEVATION
Pos.sh = Exo.shoulder;       phi12  = PHIs.el(:,1) + PHIs.el(:,2); % Sum phi's 

% Position of the upper arm and its center of mass
Pos.el_init = Pos.sh + L1 * [ cos(PHIs.el(:,3)).*cos(PHIs.el(:,1)), sin(PHIs.el(:,3)).*cos(PHIs.el(:,1)), sin(PHIs.el(:,1)) ];

% Position of the forearm and its center of mass
Pos.el2wr =  L2 * [ cos(PHIs.el(:,3)).*cos(phi12), sin(PHIs.el(:,3)).*cos(phi12), sin(phi12) ] ;        

Pos.wr_init = Pos.el_init + Pos.el2wr; % Positon of the wrist before rotation

%% Choose the mode of the function
if strcmp(mode, 'EA')
    RotMatrix = []; 
else
    % Evaluate the positions of the wrist during internal rotation of the elbow
    % Define the range of rotation (e.g., 90° leftward from the starting 
    % point) since it is a right arm. In case for the left arm it should work
    % symmetric
    RotMatrix = []; % Initialize RotMatrix
    
    for i = 1:length(Pos.wr_init) 

        % Create axis unit for RotMatrix  
        axis_unit  = (Pos.wr_init(i,:) - Pos.el_init(i,:) ) ./ norm(  Pos.wr_init(i,:)  - Pos.el_init(i,:)  );
        
        % Generate perp unit vector for the plane
        perp_vector = cross( axis_unit, [0,0,1] );   % First perpendicular vector
        perp_unit   = perp_vector ./ norm(perp_vector); % Find the perp. unit respect the axis wrist-sholder
        
        axis_gap_unit = ( Pos.el_init(i,:) - Pos.sh ) ./ norm( Pos.el_init(i,:) - Pos.sh);
    
        perp_gap_unit = cross( axis_gap_unit, [0,0,1] ) ./ norm( cross(axis_gap_unit, [0,0,1]) );
        
        circle_gap_unit = cross( perp_gap_unit, axis_gap_unit ) ./ norm( cross( perp_gap_unit, axis_gap_unit ) );
        
        % Evaluate all elbow positions for the rotation of the elbow
        for j = 1:length(Exo.theta)  
            Pos.wr_3D(i,j,:) = [Pos.el_init(i,1), Pos.el_init(i,2), Pos.el_init(i,3)] + ...
                         L1 * ( sin( Exo.theta(j) ) * axis_unit + cos( Exo.theta(j) ) * perp_unit ); % Projection along the shoulder-wrist axis 
            
            Pos.gapel_3D(i,j,:) = Pos.el_init(i,:) + ...
                         Bod.gap * ( sin( Exo.theta(j) ) * circle_gap_unit + cos( Exo.theta(j) ) * perp_gap_unit );
        
            Pos.gapsw_3D(i,j,:) = Pos.el_init(i,:) + ...
                         Bod.gap * ( sin( Exo.theta(j)-pi/2 ) * circle_gap_unit + cos( Exo.theta(j)-pi/2 ) * perp_gap_unit ); 
        end  % end for        
    end % end for
end % end if else

%% Find the position of the elbow and of the gap of the swivel angle
tau_numb = 1; % Initialize the number of torque forces for the vector
for k = 1:size(Pos.wr_3D, 1) % For each position of the elbow squeeze the pos of the wr and offsets
    wrist_position = squeeze( Pos.wr_3D(k,:,:) );
    gapsw = squeeze( Pos.gapsw_3D(k,:,:) );
    gapel = squeeze( Pos.gapel_3D(k,:,:) );
    
    for j = 1:size(Pos.wr_3D, 2) % For each position of the elbow, store positions

        Pos.wrSwivel(tau_numb,:) = wrist_position(j,:);
        Pos.elbowSwivel(tau_numb,:) = Pos.el_init(k,:); % Repeat the pos of the elbow for the position of the wrist       
        Pos.gapel(tau_numb,:) = gapel(j,:);
        Pos.gapsw(tau_numb,:) = gapsw(j,:);
        tau_numb = tau_numb + 1;

    end % end for cycle
end % end for cycle

%% Evaluate the Jacobian
Jacobian_ShEl = [];
% syms phi1 phi2 phi3 L1 L2 real
% 
% el = L1 * [cos(phi3)*cos(phi1); sin(phi3)*cos(phi1); sin(phi1)];
% phi12 = phi1 + phi2;
% wr = el + L2 * [cos(phi3)*cos(phi12); sin(phi3)*cos(phi12); sin(phi12)];
% 
% % J = [L_upper, L_fore] * [   cos(phi3) * cos(phi1),        sin(phi3) * cos(phi1),       sin(phi1);
% %                          cos(phi3) * cos(phi1 + phi2), sin(phi3) * cos(phi1+ phi2), sin(phi1 + phi2) ];
% Jacobians = jacobian(wr, [phi1, phi2 phi3]);
% 
% for i = 1:size(PHIs.J, 1)  
%     Jacobians_sub = subs(Jacobians, {phi1, phi2, phi3 L_upper, L_fore}, {PHIs.J(i, 1), PHIs.J(i, 2),q, L1, L2});
%     Jacobian_ShEl = [Jacobian_ShEl, Jacobians_sub];
% end
% Jacobian_ShEl = double(Jacobian_ShEl);

end % End of the function

%% Helpfull to take everything into GPU
% Jacobian_ShEL = gpuArray(double(Jacobian_ShEl));

% %% Create the Jacobian for the swivel angle
% Jacobian_swivel = [];
% syms elr sw_angle 
% Jacob = elr * [ cos(sw_angle), sin(sw_angle), 0 ];  % Rotation in the plane
% %Jacob = gpuArray (Jacob);
% Jacobians_sw = jacobian( Jacob, sw_angle );
% Jacobian_swivel = [];
% for j = 1:length(theta) 
%     Jacobians_SwSh = subs(Jacobians_sw, {elr, sw_angle}, {elbow_radius, theta(j)});
%     Jacobian_swivel = [Jacobian_swivel, Jacobians_SwSh];
% end
% Jacobian_swivel = double(Jacobian_swivel);
% %Jacobian_swivel = gpuArray(double(Jacobian_swivel));