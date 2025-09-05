%% Function to evaluate the torques of the exo 
function [tauvect, T, Tdist] = tauMARIONET_wr( elbow, endpoint, Actual_Pin, l0, k, robot, q)

    rVect  = endpoint - elbow; 
    lVect  = Actual_Pin - elbow;    
    
    Tdir  = lVect - rVect;      Tdist = norm( Tdir );  
    Tdir_unit  = Tdir./Tdist;   % tension direction vector 
    
    T = k * (Tdist - l0);

    tauvect = cross(rVect, T * Tdir_unit);

    % % Moentum referred to the elbow
    % tau_forearm  = cross(endpoint, T * Tdir_unit);
    % tau_upperarm = cross(Actual_Pin, -T * Tdir_unit);
    % 
    % % here evaluate the torque using inverse dynamics
    % wrench_forearm  = [tau_forearm, T * Tdir_unit];
    % wrench_upperarm = [tau_upperarm, -T * Tdir_unit];

    % fext1 = externalForce(robot, 'forearm',   wrench_forearm,  q);
    % fext2 = externalForce(robot, 'upper_arm', wrench_upperarm, q);
    % T_marionet = fext1 + fext2;
    % 
    % tauvect  = inverseDynamics( robot, q, [], [], T_marionet ); % Find the gravity torque using inverseDynamics
    
end % end function