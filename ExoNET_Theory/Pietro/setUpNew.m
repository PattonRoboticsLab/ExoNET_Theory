%% ~~ BEGIN PROGRAM: ~~
fprintf('\nSetUp parameters... \n')
global Exo Pos Bod PHIs tension TAUsDesired TauExo 
close all
TAUsDesired = struct(); TauExo = struct();     Bod = struct();
Exo = struct();         Pos = struct();

%% Pick the problem to solve
disp('Choose from menu fieldtype...')
fieldType = menu('Choose a field to approximate:', 'WeightCompensation', ...
                 'ExoEvaluation', 'ErrorAugmentation', 'EXIT');     
pause(0.2) % Fix a bug that make disappear the following menu

%% Set the parameters for the constraint
Exo.nParamsSh = 6;        Exo.nParamswr = 3;  
Exo.elementsjointSh = 2;  Exo.stiffspring = 1000;   
Exo.elementsjointwr = 1;

%% Set desired CONSTRAINTS on the parameters and create the vector pConstraint3D
% Pin for the swivel angle
     RLoHi = [.10, .11];  thetaLoHi = [pi/4, pi];  % R radius of constraint, theta angle
     yLoHi = [-.1, 0];            L0 = [0, 1];         % x distance from shoulder (consider -x), L0 resting length
perclength = [.5, 1];          stiff = [.2, 1];
Exo.numbconstraints = [.5 2.49];

% Create the constraint vector
Exo.pConstraint3D = [];  
for element = 1:Exo.elementsjointSh % For the swivel and elevation constraint
    Exo.pConstraint3D = [Exo.pConstraint3D; Exo.numbconstraints];
    for joint = 1:round(Exo.numbconstraints(2)) % For every constraint
        Exo.pConstraint3D = [Exo.pConstraint3D; RLoHi; thetaLoHi; yLoHi; L0; perclength; stiff];
    end
end
for element = 1:Exo.elementsjointwr % For the wrist bunjee
    Exo.pConstraint3D = [Exo.pConstraint3D; L0; perclength; stiff];
end

%% Initialiaze Tension and Distance variables to evaluate the torques given from the exo
Exo.K = 1000;   %N/m           % Spring Stiffness, can be changed depending on the spring
Exo.stretch_ratio_limit = 2;   % Double of the length, can be changed depending on the spring

%% Tension for Yaseen's Quarter Inch Bungee Cords
tension = @(L0,L) ((710.5*(L/L0).^5-5442*(L/L0).^4+1.654e4*(L/L0).^3-2.495e4*(L/L0).^2+1.871e4*(L/L0)-5575)).*((L/L0)>1);

%% Body
Bod.M = 80;                     % Body mass 
Bod.L = [.28 .25;];             % Segment lengths (humerous, arm)
Bod.gap = .045;                 % Average radius of the forearm
Bod.R = [.45 .5];               % Proximal to centers of mass of segments referred to actual length
Bod.pose = pi/180*[-97 70 45];  % Token body position (can be anything)

%% Setup span of full workspace posture evaluation points (angles) for ELEVATION
Bod.nAngles = 3;     % # shoulder & elbow angles in a span for evaluation
Bod.nAngles_z = 4;   % # decided to set different number from x and y in order to occupy better the space
Bod.n_angles_3D = 3; % # angles for rotation of the shoulder

phi1 = pi/180*linspace(  290+10^-6, 350+10^-6, Bod.nAngles );    % Phi1 see the figure above
phi2 = pi/180*linspace(  10+10^-6,  90+10^-6,  Bod.nAngles );    % Phi2 see the figure above
phi3 = pi/180*linspace(  0,         120,       Bod.nAngles_z );  % Phi3 see the figure above 3D, from rest position when relaxed to max flexion PB 
theta = pi/180*linspace( 0,         90,        Bod.n_angles_3D );% theta rotation

PHIs.el = zeros( length(phi3)*length(phi1)*length(phi2)*length(theta), 3); % Initialize the vector of the angles for elevation

% Nested 4-loop establishes grid of phi's moving, this dictates the movement of the arm in the space
for i=1:length(phi3)         % For every angle of the sagital plane 
    for j=1:length(phi1)     % Flex the shoulder
        for t=1:length(theta)
            PHIs.el = [PHIs.el; phi1(j), phi3(i), theta(t)]; % Store all the angles
        end
    end
end

%% Collection of some variables into the GPU for speeding up process
%PHIs = gpuArray(PHIs);   Bod = gpuArray(Bod);     Exo = gpuArray(Exo);

%% Use ForwardKin3D to find the position of the arm joints in the space
[Pos, RotMatrix, Jacobian_ShEL, Jacobian_swivel] = forwardKin3D(PHIs, Bod);  

%% Plot all the figures of the movement of the shoulder to see if they are correct
% Here the three images to show the 3 perspectives
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
%% FIGURE 2
% put_fig(2,.9,.35,.6,.5); grid on; xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal; % 3D view of the arm in the space
% drawBody3D(Bod)
% plot3(Pos.wr(:,1),Pos.wr(:,2),Pos.wr(:,3),'.','color',.7*[1 1 1])
% plot3(Pos.el(:, 1), Pos.el(:, 2), Pos.el(:, 3), 'g-', 'LineWidth', 1, 'DisplayName', 'gap swivel');
% plot3(Pos.swgap(:, 1), Pos.swgap(:, 2), Pos.swgap(:, 3), 'r-', 'LineWidth', 1, 'DisplayName', 'gap swivel');
% plot3(Pos.elgap(:, 1), Pos.elgap(:, 2), Pos.elgap(:, 3), 'b-', 'LineWidth', 1, 'DisplayName', 'gap elevation');
% plot3(0, 0, 0, 'ro', 'MarkerSize', 4, 'LineWidth', 1.5, 'DisplayName', 'Shoulder');
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

