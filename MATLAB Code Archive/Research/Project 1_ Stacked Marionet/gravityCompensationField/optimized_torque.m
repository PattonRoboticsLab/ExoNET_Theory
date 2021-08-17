function optimized_torque(Lengths, Weights, shoulder_angle, elbow_angle, maxima)
    
    global phi desired_torque n_stackedMarionet y spring L
    
    upperArm_weight = Weights(1);
    foreArm_weight = Weights(2);
    hand_weight = Weights(3);
    
    L_UpperArm = Lengths(1);
    L_ForeArm = Lengths(2);
    
    options.MaxIter = Inf;
    options.MaxFunEvals = Inf;
    init_param = rand(1,(n_stackedMarionet*3));
    
    for i = 1:length(shoulder_angle)-1
        
        R_el = [cos(shoulder_angle(i)), -sin(shoulder_angle(i)); sin(shoulder_angle(i)), cos(shoulder_angle(i))];
        sh_pos = [0; 0];
        el_pos = sh_pos+(R_el*[L_UpperArm; 0]);
        
        alpha = shoulder_angle(i)+elbow_angle;

        desiredTau_el = (foreArm_weight/2+hand_weight)*L_ForeArm.*cos(alpha);
        desiredTau_sh = (upperArm_weight/2+foreArm_weight+hand_weight)*L_UpperArm*cos(shoulder_angle(i))+desiredTau_el;
        
        %optimization elbow torque
        L = L_ForeArm;
        phi = elbow_angle';
        y = @(p, phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); %arm for the force 
        spring = @(p, phi) L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi); %spring length
        starting_torque = stacks(init_param, phi); %torque with random guess
        mean_err_old_el = mean(abs(desiredTau_el' - starting_torque));
        desired_torque = desiredTau_el';
        optimalParam_el = minimize_cost(init_param, options, mean_err_old_el);
        optimTorque_el = stacks(optimalParam_el, phi);
        
        %optimization shoulder torque
        L = L_UpperArm;
        phi = shoulder_angle';
        y = @(p, phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); %arm for the force 
        spring = @(p, phi) L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi); %spring length
        starting_torque = stacks(init_param, phi); %torque with random guess
        mean_err_old_sh = mean(abs(desiredTau_sh' - starting_torque));
        desired_torque = desiredTau_sh';
        optimalParam_sh = minimize_cost(init_param, options, mean_err_old_sh);
        optimTorque_sh = stacks(optimalParam_sh, phi);
        
        phi = elbow_angle';
        
        for j = 1:length(elbow_angle)
            
            angles = [shoulder_angle(i); elbow_angle(j)];
            torque_vector = [optimTorque_sh(j); optimTorque_el(j)];
            jacobian_matrix = jacobian(angles, Lengths);

            force_vector = ((jacobian_matrix)')\torque_vector;

            force_x = force_vector(1);
            force_y = force_vector(2);
            
            x_force_norm = (force_x/maxima(i, j))*.15;
            y_force_norm = (force_y/maxima(i, j))*.15;
%             if (abs(force_y) > abs(force_x) && force_y ~= 0)
%                 x_force_norm = (force_x/abs(force_y))*.15;
%                 y_force_norm = (force_y/abs(force_y))*.15;
%             elseif (abs(force_y) < abs(force_x) && force_x ~= 0)
%                 x_force_norm = (force_x/abs(force_x))*.15;
%                 y_force_norm = (force_y/abs(force_x))*.15;
%             elseif (force_x == 0 && force_y == 0)
%                 x_force_norm = 0;
%                 y_force_norm = 0;
%             end
            
            R_wr = [cos(alpha(j)), -sin(alpha(j)); sin(alpha(j)), cos(alpha(j))];
            wrist_position = el_pos+(R_wr*[L_ForeArm; 0]);
            
            arrow(wrist_position, wrist_position+[x_force_norm; y_force_norm], 'Color', 'k', 'Linewidth', 2);
            hold on
            
            
        end
                
    end
    
end