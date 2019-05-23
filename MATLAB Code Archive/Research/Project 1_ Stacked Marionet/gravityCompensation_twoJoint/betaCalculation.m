function beta = betaCalculation(Lengths, elbow_angle, n_el)
    
    L_UpperArm = Lengths(1);
    L_ForeArm = Lengths(2);

    sh_pos = [0; 0]; 
    el_posBeta = sh_pos+([L_UpperArm; 0]);
    beta = zeros(n_el, 1);
    wrist_position = zeros(2, n_el);
    for i = 1:n_el

        R_wr = [cos(elbow_angle(i)), -sin(elbow_angle(i)); sin(elbow_angle(i)), cos(elbow_angle(i))];
        wrist_position(:, i) = el_posBeta+(R_wr*[L_ForeArm; 0]); 
        beta(i) = atan2(wrist_position(2, i), wrist_position(1, i));

    end


end