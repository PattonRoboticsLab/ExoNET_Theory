% Calcuate the desired torques needed from weight cancellation
% Patton 2019-01-14
function [TAUsDesired, MaxTorques, robot, q ] = weightEffect3D( Bod, Pos, Exo )
    global ProjectName
    fprintf('\n - %s : - \n', ProjectName)

    %% Evaluation of the torque for swivel angle
    Tau_shoulder = zeros(length(Pos.wrSwivel),3);
       Tau_elbow = zeros(length(Pos.wrSwivel),3);
    
    g = 9.81;  
    
    for i = 1:size(Pos.wrSwivel, 1) % For each position of the elbow
    
        % Position of the CM of the forearm and upperarm
        upperarm_CM = Pos.sh                + ( Pos.elbowSwivel(i, :) - Pos.sh  )            * Bod.R(1) ; 
        forearm_CM  = Pos.elbowSwivel(i, :) + ( Pos.wrSwivel(i, :) - Pos.elbowSwivel(i, :) ) * Bod.R(2) ; 
    
        % Torque force acting on the shoulder and elbow
        Tau_Sh = cross( forearm_CM - Pos.sh,  [ 0, 0, -( Bod.weights(2) + Bod.weights(3) ) ] * g ) + ...
                 cross( upperarm_CM - Pos.sh, [ 0, 0,           - Bod.weights(1)           ] * g );
    
        Tau_El = cross( forearm_CM - Pos.elbowSwivel(i, :),  [ 0, 0, -( Bod.weights(2) + Bod.weights(3) ) ] * g );
    
        % Store the torques
        Tau_shoulder(i, :) = - Tau_Sh;     Tau_elbow(i,:) = - Tau_El;
    end
    
    %% Store the Tau_desired for the evaluation, these are the gold standards
    TAUsDesired.TauSh_tot = Tau_shoulder;   TAUsDesired.TauEl_tot = Tau_elbow;
    
    %% Plot the force fields 
    % figure;
    %plotVectField3D( q, Bod, Pos, Exo, robot, TAUsDesired.TauSh_tot, TAUsDesired.TauEl_tot, 'b', 2,   3,   2 );
   
    %% Plot animation of the arm in the two solutions
    % Helpful to understand better the movement of the arm and if everything is
    % working rightfully   
    %plotanimation(Pos.wrSwivel, Pos.elbowSwivel, Pos);

    %% Plot the animation of the arm
    % Poi testa la funzione
    % test_joint_angles(robot);
    % debug_arm_frames(robot, q, [0,0,0], Pos.elbowSwivel, Pos.wrSwivel);

    %animateArmMotion(robot, q);

    %% Create a vector Maxtorques with the maximum torque and the position of the elbow and wrist 
    % May be helpful, or maybe not
    [Sh_torquemax,   IndexSh] = max( vecnorm( TAUsDesired.TauSh_tot, 2, 2 ));
    [El_torquemax,   IndexEl] = max( vecnorm( TAUsDesired.TauEl_tot, 2, 2 ));
    
    % pos elbow, pos wrist, taux, tauy, tauz for both movements
    MaxTorques = [Pos.elbowSwivel(IndexSh,1), Pos.elbowSwivel(IndexSh,2), Pos.elbowSwivel(IndexSh,3),   Pos.wrSwivel(IndexSh,1), Pos.wrSwivel(IndexSh,2), Pos.wrSwivel(IndexSh,3),  Sh_torquemax;
                  Pos.elbowSwivel(IndexEl,1), Pos.elbowSwivel(IndexEl,1), Pos.elbowSwivel(IndexEl,1),   Pos.wrSwivel(IndexEl,1), Pos.wrSwivel(IndexEl,2), Pos.wrSwivel(IndexEl,3),  El_torquemax];
    
    %% Torque with the rigidBody3D
    % q contains yaw pitch roll and wrist
    nSamples = length(Pos.elbowSwivel);    q = zeros(nSamples, 4);
    robot = create3DArmModel( Bod.L(1), Bod.L(2), Bod ); % Create the model of the robot

    for i=1:nSamples
        q(i,:) = computeJointAngles3D( Pos.sh, Pos.elbowSwivel(i,:), Pos.wrSwivel(i,:));
        %q(i,:) = estimateJointAnglesFromPositions(robot, Pos.sh, Pos.elbowSwivel(i,:), Pos.wrSwivel(i,:));
    end

end % End of the function



