function  [bestParam, averageErr_el, averageErr_sh, averagePerc_el, averagePerc_sh, Rsquared_el, Rsquared_sh] =...
    gravityProcess_twoJoints4param(n_angles, L_UpperArm, L_ForeArm, Mass_TotalBody)

    phi1 = linspace(-pi/2, 0, n_angles);
    phi2 = linspace(pi/36, 2*pi/3, n_angles);
    
    %% Choosing parameters
    if (mod(n_angles, 2) == 0)
        shoulder_angle = phi1(n_angles/2-1);
        elbow_angle = phi2(n_angles/2+1);
    else
        shoulder_angle = phi1(n_angles/2-.5);
        elbow_angle = phi2(n_angles/2+.5);
    end
    
    alpha = shoulder_angle+elbow_angle;

    %% Finding Weights
    g = 9.81;   %gravity constant
    mass_Upper = (2.71/100)*Mass_TotalBody; %mass upper arm
    mass_Fore = (1.62/100)*Mass_TotalBody;  %mass forearm
    mass_Hand = (0.61/100)*Mass_TotalBody;  %mass hand
    hand_weight = mass_Hand*g;
    foreArm_weight = mass_Fore*g;
    upperArm_weight = mass_Upper*g;
    Weights = [upperArm_weight, foreArm_weight, hand_weight];
    
    %% Draw image

    R_el = [cos(shoulder_angle), -sin(shoulder_angle); sin(shoulder_angle), cos(shoulder_angle)];
    R_wr = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];
    Lengths = [L_UpperArm, L_ForeArm];
    sh_pos = [0; 0];
    el_pos = sh_pos+(R_el*[L_UpperArm; 0]);
    wr_pos = el_pos+(R_wr*[L_ForeArm; 0]);
    
    draw_system(sh_pos, el_pos, wr_pos, Weights, R_el);
    
    [bestParam, averageErr_el, averageErr_sh, averagePerc_el, averagePerc_sh, Rsquared_el, Rsquared_sh] =...
        optimizedTorque_twoJoints4param(Lengths, Weights, phi1, phi2);

end
