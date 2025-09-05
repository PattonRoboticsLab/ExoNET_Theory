% % Set as many parameters as possible for the stacked marionet applications
% Patton 2019-01-11
%  !!!y axis in front of the person, x axis on the lateral side, z axis is the
%  height!!!
% 
%       | PHI2  .                               |      .
%       |      .                                |     .
%       |    .                                  |    . 
%       |   .                                   |   .
%       |  .                                    |  .
%       | .                                     | .
%       o                                       o
%      /                                       /  
%     /                                       /   
%    /                                       /  
%   /   PHI 1                               /   PHI 3  
%  o . . . . . . .   lateral view          o . . . . . . .   upper view 
%
%                             .X
%                       .   X  
%         TVect   .      X       lVect
%           .         X
%       o          X       
%   r  /        X
%     /      X       phi
%    /   theta        
%   /  X            
%  O .  .  .  .  .  .  .  .  .  .  .  . 
%aaaaaaaaaaaaaaaaaaaaaaaa
%% ~~ BEGIN PROGRAM: ~~
fprintf('\nSetUp parameters... \n')
global Exo Pos Bod PHIs tension TAUsDesired TauExo ProjectName
close all
Bod = struct();    Exo = struct();    Pos = struct();
TAUsDesired = struct();      TauExo = struct();

%% Pick the problem to solve
disp('Choose from menu fieldtype...')
ProjectName = menu('Choose a field to approximate:', 'WeightCompensation', ...
                 'ExoEvaluation', 'ErrorAugmentation', 'Analyze Exo', 'EXIT');     
pause(0.2) % Fix a bug that make disappear the following menu

%% Set the parameters for the constraint
Exo.nParamsSh = 9;         Exo.nParamswr = 3;  
Exo.elementsjointSh = 2;   Exo.elementsjointwr = 1;  
Exo.shoulder = [0.1855, -0.00435, 1.35];

%% Set desired CONSTRAINTS on the parameters and create the vector pConstraint3D
% Pin for the swivel angle
 RLoHi = [.06, .07];       thetaLoHi = [pi/2-pi/6, pi-pi/6];  % R radius of constraint, theta angle
 xLoHi = [-.10, .01];     perclength = [0, 1];           % x distance from shoulder (consider -x), L0 resting length
 stiff = [0, 1];           L0_recoil = [0, 1];
    L0 = [0, 1];            K_recoil = [0, 1];
              DL_pre_extended_recoil = [0, 1];

Exo.numbconstraints = [0, 2];

% Create the constraint vector P
Exo.P = [];  
for element = 1:Exo.elementsjointSh % For the swivel and elevation constraint    
    Exo.P = [ Exo.P; Exo.numbconstraints ];

    for joint = 1:round( Exo.numbconstraints(2) ) % For every constraint
        Exo.P = [ Exo.P; RLoHi; thetaLoHi; xLoHi; L0; perclength; stiff; K_recoil; L0_recoil; DL_pre_extended_recoil ];
    end

end

for element = 1:Exo.elementsjointwr % For the wrist bunjee
    Exo.P = [Exo.P; L0; perclength; stiff];
end

%% Initialiaze Tension and Distance variables to evaluate the torques given from the exo
Exo.K = 1000;     %N/m                % Spring Stiffness, can be changed depending on the spring
Exo.stretch_ratio_limit = 2;          % Double of the length, can be changed depending on the spring
Exo.stretch_ratio_limit_recoil = 2;   % Double of the length, can be changed depending on the spring
Exo.L_rail = .15;                     % 4N max preload of the spring for recoil

%% Tension for Yaseen's Quarter Inch Bungee Cords
tension = @(L0,L) ((710.5*(L/L0).^5-5442*(L/L0).^4+1.654e4*(L/L0).^3-2.495e4*(L/L0).^2+1.871e4*(L/L0)-5575)).*((L/L0)>1);

%% Body
Bod.M   = 80;             % Body mass 
Bod.L   = [.28 .25;];     % Segment lengths (humerous, arm)
Bod.gap = .045;           % Average radius of the forearm
Bod.R   = [.45 .5];       % Proximal to centers of mass of segments referred to actual length

%% Body weights (there's the possibility to add 3kg on the hand to simulate hemiparesis)
hand_weight       = (0.61/100) * Bod.M;  % 0.226796*g;  %(0.61/100)*Bod.M*g;   % from Winter's book
foreArm_weight    = (1.62/100) * Bod.M;  % 0.408233*g;  %(1.62/100)*Bod.M*g;
upperArm_weight   = (2.71/100) * Bod.M;  % (0.453592+0.317515)*g;  %(2.71/100)*Bod.M*g;
hemiparesis_mass  = 3;                   % Can be changed to 3kg to add hemiparesis
Bod.weights       = [upperArm_weight, foreArm_weight, hand_weight + hemiparesis_mass]; % Use a variable to attach weights of the various parts of the body

%% Setup span of full workspace posture evaluation points (angles) for ELEVATION
Bod.nAngles = 6;     % # phi1 elevation upperarm
Bod.nAngles2 = 1;    % # phi2 flexion elbow
Bod.nAngles_z = 6;   % # elevation planes
Bod.n_angles_3D = 2; % # humeral rotation

phi1 = pi/180*linspace( 280+10^-6, 400+10^-6,  Bod.nAngles );   % Phi1 shoulder elevation
phi2 = pi/180*linspace(   5+10^-6,   5+10^-6,  Bod.nAngles2 );  % Phi2 elbow flexion
phi3 = pi/180*linspace(  10+10^-6, 110+10^-6,  Bod.nAngles_z ); % Phi3 plane of elevation

PHIs.el = [];  % Initialize the vector of the angles for elevation

% Nested 3-loop establishes grid of phi's moving, this dictates the movement of the arm in the space
for i=1:length(phi3)         % For every angle of the sagital plane 
    for j=1:length(phi1)     % Flex the shoulder
        for k=1:length(phi2) % Flex the elbow
            PHIs.el = [PHIs.el; phi1(j), phi2(k), phi3(i)]; % Store all the angles
        end 
    end
end

PHIs.J = [];
% PHIs to evaluate the Jacobian
for i=1:length(phi3)
    for j=1:length(phi1)     % Flex the shoulder
        for k=1:length(phi2) % Flex the elbow           
            for angle=1:Bod.n_angles_3D
                PHIs.J = [PHIs.J; phi1(j), phi2(k)]; % Store all the angles
            end           
        end 
    end
end

%% Set angles for rotation
Exo.theta = pi/180*linspace( 95, 185, Bod.n_angles_3D ); % Limited swivel angle 

%% Collection of some variables into the GPU for speeding up process
%PHIs = gpuArray(PHIs);   Bod = gpuArray(Bod);     Exo = gpuArray(Exo);

%% Use ForwardKin3D to find the position of the arm joints in the space
[Pos, RotMatrix, Jacobian_ShEl] = forwardKin3D(PHIs, Bod, Exo, Pos, '');  

%% Plot all the figures of the movement of the shoulder to see if they are correct
% Here the three images to show the 3 perspectives

%% FIGURE 3D
% put_fig(2,.9,.35,.6,.5); grid on; xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal; % 3D view of the arm in the space
% drawBody3D(Bod);
% plot3(Pos.wr(:,1),Pos.wr(:,2),Pos.wr(:,3),'.','color',.7*[1 1 1])
% plot3(Pos.el(:, 1), Pos.el(:, 2), Pos.el(:, 3), 'g-', 'LineWidth', 1, 'DisplayName', 'gap swivel');
% plot3(Pos.swgap(:, 1), Pos.swgap(:, 2), Pos.swgap(:, 3), 'r-', 'LineWidth', 1, 'DisplayName', 'gap swivel');
% plot3(Pos.elgap(:, 1), Pos.elgap(:, 2), Pos.elgap(:, 3), 'b-', 'LineWidth', 1, 'DisplayName', 'gap elevation');
% plot3(0, 0, 0, 'ro', 'MarkerSize', 4, 'LineWidth', 1.5, 'DisplayName', 'Shoulder');
% axis equal;

%% FIGURE 1
% put_fig(fieldType,.9,.35,.6,.5); 
% subplot(1,3,1);  grid on; xlabel('X'); ylabel('Z'); axis equal; % place figure of the referred field type LATERAL view
% drawBody2_lat(Bod.pose,Bod);     hold on     % lateral view
% scatter(Pos.wr(:,1),Pos.wr(:,3),'.','color',.7*[1 1 1]); 
% fprintf(' parameters set. \n ')
% 
% subplot(1,3,2);grid on; xlabel('X'); zlabel('Y'); axis equal; % UPPER view
% drawBody2_up(Bod.pose,Bod); hold on %upper view
% scatter(Pos.wr(:,1),Pos.wr(:,2),'.','color',.7*[1 1 1]);
% axis equal
% 
% subplot(1,3,3); grid on; ylabel('Y'); zlabel('Z'); axis equal;% BACK view
% drawBody2_back(Bod.pose,Bod); hold on %back view
% scatter(Pos.wr(:,2),Pos.wr(:,3),'.','color',.7*[1 1 1]);
% axis equal

%% FIGURE 3
% put_fig(3,.9,.35,.6,.5); % view to se the path of the elbow
% % Plot wrist positions
% scatter3(Pos.wrsw(:, 1), Pos.wrsw(:, 2), Pos.wrsw(:, 3),5,'k','filled'); hold on
% plot3(0, 0, 0, 'ro', 'MarkerSize', 4, 'LineWidth', 1.5, 'DisplayName', 'Shoulder');
% % Plot elbow positions for all wrist configurations
% for i = 1:size(Pos.el_3D)
%     elbow_positions = squeeze(Pos.el_3D(i, :, :));  % Extract elbow positions for the current wrist
%     gapsw = squeeze(Pos.gap_swivelsw(i, :, :));
%     gapel = squeeze(Pos.gap_swivelel(i, :, :));
%     % Plot trajectory as a 3D line
%     plot3(elbow_positions(:, 1), elbow_positions(:, 2), elbow_positions(:, 3), 'g-', 'LineWidth', 1, 'DisplayName', 'Elbow Path');
%     % Plot individual elbow points with smaller dots
%     scatter3(elbow_positions(:, 1), elbow_positions(:, 2), elbow_positions(:, 3), 5, 'g', 'filled');  % Smaller dots
%     plot3(gapsw(:, 1), gapsw(:, 2), gapsw(:, 3), 'r-', 'LineWidth', 1, 'DisplayName', 'gap swivel');
%     plot3(gapel(:, 1), gapel(:, 2), gapel(:, 3), 'b-', 'LineWidth', 1, 'DisplayName', 'gap elevation');
% end
% view(3); hold off; 

