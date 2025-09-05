%% Function to evaluate the torques of the exo 
function [tauvect, T, Tdist] = tauMARIONET3D( shoulder, endpoint, Actual_Pin, l0, k, k_recoil, l0_recoil, Lrail, DLmax, L_pre_extended_recoil)
       
    % L_pre_extended_recoil is the length of the spring that remains fixed
    % in order to set a threshold of force
    rVect = endpoint - shoulder;  lVect = Actual_Pin - shoulder;        
     Tdir = lVect - rVect;        Tdist = sqrt(sum(Tdir.^2, 2));  
    
    Tdir_unit  = Tdir ./ Tdist;
    
   %% -------------------------- Recoil system ------------------------------
    T = 0; % Set Tension to zero
    pre_extension = l0 - (Lrail - L_pre_extended_recoil) - Tdist; % Evaluate if the spring is slacked

    if pre_extension < 0  % Spring not slacked
        T = - k * pre_extension; % Tension fo the spring 
        Tmin = - k_recoil * (l0_recoil - L_pre_extended_recoil); % Evaluate the minimun force to have the recoil system activated
            
        if l0_recoil <= L_pre_extended_recoil % Recoil is pre-extended    
            if T > Tmin
                k_eq = (k * k_recoil) / (k + k_recoil); % Evaluate the equal stiffness
                T = - k_eq * (l0 + l0_recoil - Tdist - Lrail); % Evaluate the tension using the stiffness K_eq
            end
        end

    end
    
    tauvect = cross(rVect, T .* Tdir_unit);

    % % Create the wrench, momentums plus force vecotrs
    % wrench_marionet = [cross(rVect, T .* Tdir_unit), T .* Tdir_unit];
    % 
    % % Set BodyNames as in the robot
    % bodynames = {'shoulder_abduction','humeral_elevation','humeral_rotation','upper_arm'};
    % 
    % T_marionet_total = externalForce(robot, bodynames{1}, wrench_marionet, q);
    % 
    % parfor i = 2:length(bodynames)
    %     T_i = externalForce(robot, bodynames{i}, wrench_marionet, q);  
    %     T_marionet_total(i,:)= T_i(i,:);
    % end
    % 
    % tauvect = inverseDynamics(robot, q, [], [], T_marionet_total);

end % end function