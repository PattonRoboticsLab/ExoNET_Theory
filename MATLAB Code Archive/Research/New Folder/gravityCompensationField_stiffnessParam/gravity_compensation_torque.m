    function [torque_vector, force_vector, weights]= gravity_compensation_torque(L_UpperArm, L_ForeArm, Mass_TotalBody,...
                                                                shoulder_angle, elbow_angle)
                                                          
    if (shoulder_angle > pi/2 || shoulder_angle < -pi/2)
        error('The chosen shoulder angle is out of the range of motion, choose an angle between -pi/2 and pi/2!');
    end
    if(elbow_angle > 2*pi/3 || elbow_angle < 0)
        error('The chosen elbow angle is out of the range of motion, choose an angle between 0 and 2*pi/3!');
    end
    g = 9.81;   %gravity constant
    mass_Upper = (2.71/100)*Mass_TotalBody; %mass upper arm
    mass_Fore = (1.62/100)*Mass_TotalBody;  %mass forearm
    mass_Hand = (0.61/100)*Mass_TotalBody;  %mass hand
    hand_weight = mass_Hand*g;
    foreArm_weight = mass_Fore*g;
    upperArm_weight = mass_Upper*g;
    weights = [upperArm_weight, foreArm_weight, hand_weight];
    lengths = [L_UpperArm; L_ForeArm];
    angles = [shoulder_angle; elbow_angle];
    alpha = shoulder_angle+elbow_angle;

    elbow_torque = (foreArm_weight/2+hand_weight)*L_ForeArm*cos(alpha);
    shoulder_torque = (upperArm_weight/2+foreArm_weight+hand_weight)*L_UpperArm*cos(shoulder_angle)+elbow_torque;
    
    torque_vector = [shoulder_torque; elbow_torque];
    jacobian_matrix = jacobian(angles, lengths);
    
    force_vector = ((jacobian_matrix)')\torque_vector;
   
end