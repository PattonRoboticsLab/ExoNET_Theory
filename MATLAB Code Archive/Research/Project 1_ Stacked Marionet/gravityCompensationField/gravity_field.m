function gravity_field(Lengths, Weights, shoulder_angle, elbow_angle) 
        
    %Weights: [upper arm weight, fore arm weight, hand weight]
    %Lenghts: [upper arm length, fore_arm length]
    
    upperArm_weight = Weights(1);
    foreArm_weight = Weights(2);
    hand_weight = Weights(3);
    
    L_UpperArm = Lengths(1);
    L_ForeArm = Lengths(2);
    maxima = zeros(length(shoulder_angle)-1, length(elbow_angle));
    
    for i = 1:length(shoulder_angle)-1
        
        R_el = [cos(shoulder_angle(i)), -sin(shoulder_angle(i)); sin(shoulder_angle(i)), cos(shoulder_angle(i))];
        sh_pos = [0; 0];
        el_pos = sh_pos+(R_el*[L_UpperArm; 0]);
        
        for j = 1:length(elbow_angle)
            
            alpha = shoulder_angle(i)+elbow_angle(j);
            angles = [shoulder_angle(i); elbow_angle(j)];
            
            elbow_torque = (foreArm_weight/2+hand_weight)*L_ForeArm*cos(alpha);
            shoulder_torque = (upperArm_weight/2+foreArm_weight+hand_weight)*L_UpperArm*cos(shoulder_angle(i))+elbow_torque;

            torque_vector = [shoulder_torque; elbow_torque];
            jacobian_matrix = jacobian(angles, Lengths);

            force_vector = ((jacobian_matrix)')\torque_vector;
            
            force_x = force_vector(1);
            force_y = force_vector(2);
            
            if abs(force_y) > abs(force_x)
                x_force_norm = (force_x/abs(force_y))*.15;
                y_force_norm = (force_y/abs(force_y))*.15;
                maxima(i, j) = abs(force_y);
            else
                x_force_norm = (force_x/abs(force_x))*.15;
                y_force_norm = (force_y/abs(force_x))*.15;
                maxima(i, j) = abs(force_x);
            end
            
            R_wr = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];
            wrist_position = el_pos+(R_wr*[L_ForeArm; 0]);
            arrow(wrist_position, wrist_position+[x_force_norm; y_force_norm], 'Color', 'r', 'Linewidth', 4);
            hold on
            
        end
    end


end