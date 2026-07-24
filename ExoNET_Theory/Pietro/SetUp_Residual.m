Bod = struct();          Exo = struct();     Pos = struct();
TAUsDesired = struct();  TauExo = struct();

%% Pick the problem to solve
disp('Choose from menu fieldtype...')
ProjectName = menu('Choose a field to approximate:', 'WeightCompensation', ...
                 'ExoEvaluation', 'ErrorAugmentation', 'Analyze Exo', 'EXIT');     
pause(0.2) % Fix a bug that make disappear the following menu

%% Set the parameters for the constraint
Exo.nParamsSh = 4;         Exo.nParamswr = 2;  
Exo.elementsjointSh = 2;   Exo.elementsjointwr = 1;  
Exo.shoulder = [0.1855, -0.00435, 1.35];

%% Set desired CONSTRAINTS on the parameters and create the vector pConstraint3D
% Pin for the swivel angle
 RLoHi = [.1, .11];     thetaLoHi = [pi/2, pi];  % R radius of constraint, theta angle
 xLoHi = [-.15, .03];  perclength = [0.8, 1.2]; % x distance from shoulder (consider -x), L0 resting length
    L0 = [0, 1];  

 stiff = [0, 1]; 
 % L0_recoil = [0, 1];   K_recoil = [0, 1];    DL_pre_extended_recoil = [0, 1];

Exo.numbconstraints = [0, 3];

% Create the constraint vector P
Exo.P = [];  
for element = 1:Exo.elementsjointSh % For the swivel and elevation constraint    
    Exo.P = [ Exo.P; Exo.numbconstraints ];
    for joint = 1:round( Exo.numbconstraints(2) ) % For every constraint
        Exo.P = [ Exo.P; RLoHi; thetaLoHi; xLoHi; L0 ]; %; stiff]; % K_recoil; L0_recoil; DL_pre_extended_recoil ];
    end
end

for element = 1:Exo.elementsjointwr % For the wrist bunjee
    Exo.P = [Exo.P; L0; perclength];
end

%% Initialiaze Tension and Distance variables to evaluate the torques given from the exo
Exo.K = 1000;     %N/m                % Spring Stiffness, can be changed depending on the spring
Exo.stretch_ratio_limit = 2.5;        % Double of the length, can be changed depending on the spring
%Exo.stretch_ratio_limit_recoil = 2;  % Double of the length, can be changed depending on the spring
%Exo.L_rail = .15;                    % 4N max preload of the spring for recoil

%% Body
Bod.M   = 80;             % Body mass 
Bod.L   = [.28 .25;];     % Segment lengths (humerous, arm)
Bod.gap = .045;           % Average radius of the forearm
Bod.R   = [.45 .5];       % Proximal to centers of mass of segments referred to actual length

%% Body weights (there's the possibility to add 3kg on the hand to simulate hemiparesis)
hand_weight       = (0.61/100) * Bod.M;  % 0.226796*g;  %(0.61/100)*Bod.M*g;   % from Winter's book
foreArm_weight    = (1.62/100) * Bod.M;  % 0.408233*g;  %(1.62/100)*Bod.M*g;
upperArm_weight   = (2.71/100) * Bod.M;  % (0.453592+0.317515)*g;  %(2.71/100)*Bod.M*g;
hemiparesis_mass  = 0;                   % Can be changed to 3kg to simulate hemiparesis
Bod.weights       = [upperArm_weight, foreArm_weight, hand_weight + hemiparesis_mass]; % Use a variable to attach weights of the various parts of the body

%% Setup span of full workspace posture evaluation points (angles) for ELEVATION
Bod.nAngles = 10;     % # phi1 elevation upperarm
Bod.nAngles2 = 2;     % # phi2 flexion elbow
Bod.nAngles_z = 2;    % # elevation planes
Bod.n_angles_3D = 10; % # humeral rotation

phi1 = pi/180*linspace( -80+10^-6, 10+10^-6,  Bod.nAngles );   % Phi1 shoulder elevation
phi2 = pi/180*linspace(   0+10^-6, 60+10^-6, Bod.nAngles2 );  % Phi2 elbow flexion
phi3 = pi/180*linspace(   0+10^-6, 90+10^-6,  Bod.nAngles_z ); % Phi3 plane of elevation

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
Exo.theta = pi/180*linspace( 0, 90, Bod.n_angles_3D ); % Limited swivel angle 

%% Collection of some variables into the GPU for speeding up process
%PHIs = gpuArray(PHIs);   Bod = gpuArray(Bod);     Exo = gpuArray(Exo);

%% Use ForwardKin3D to find the position of the arm joints in the space
[Pos, RotMatrix, Jacobian_ShEl] = forwardKin3D_Residual(PHIs, Bod, Exo, Pos, '');  

