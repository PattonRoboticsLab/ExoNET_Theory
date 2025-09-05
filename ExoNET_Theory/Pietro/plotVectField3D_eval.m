function [Force3D] = plotVectField3D_eval(Bod, Pos, TauSh_tot, TauEl_tot, Rot_matrix, Angles, Colr)
    %% Evaluate the Jacobian and the force field in 3D
    syms L2 L1 phi1 phi2 flex 

    J2 = [L1, L2] * [cos(phi1) * cos(phi2), sin(phi1) * cos(phi2), sin(phi2); ...
                     cos(phi1 + flex) * cos(phi2), sin(phi1 + flex) * cos(phi2), sin(phi2)];

    Jacobians = jacobian(J2, flex);
    
    % Set up the figure
    figure;
    % Evaluate for every position the force at the end effector
    for i = 1:size(Pos.EvalWr, 1)
        % Substitute the values of lengths and flexion angles into the Jacobian
        Jacobians_sub = subs(Jacobians, {L2, L1, phi1, phi2, flex}, ...
                             {Bod.L(2), Bod.L(1), Angles.horizontal_adduction_angle, Angles.abduction_angle, Angles.flexion_angles(i)});
       
        Force3D = double(Jacobians_sub' * [TauSh_tot(i,:)', TauEl_tot(i,:)']);

        % Normalize the Force Vectors to handle the singularity position and plot them
        Force3D_tot = Force3D * 0.02; % Scale factor
        
        % Plot the end effector (wrist) and force vectors
        scatter3(Pos.EvalWr(i, 1), Pos.EvalWr(i, 2), Pos.EvalWr(i, 3), 5, 'k', 'filled'); hold on;
        scatter3(Pos.EvalEl(1, 1), Pos.EvalEl(1, 2), Pos.EvalEl(1, 3), 5, 'k', 'filled');
        
        %quiver3(Pos.EvalEl(1, 1), Pos.EvalEl(1, 2), Pos.EvalEl(1, 3), 0, 0, abs(Force3D_tot(1)), ...
        %        'Color', Colr, 'LineWidth', 1.5, 'MaxHeadSize', 2);

        quiver3(Pos.EvalWr(i, 1), Pos.EvalWr(i, 2), Pos.EvalWr(i, 3), 0, 0, abs(Force3D_tot(1)), ...
                'Color', Colr, 'LineWidth', 1.5, 'MaxHeadSize', 2.3);

    end

    % Plot the shoulder and elbow positions
    scatter3(Pos.sh(1), Pos.sh(2), Pos.sh(3), 'g', 'filled', 'DisplayName', 'Shoulder');
    scatter3(Pos.EvalEl(1), Pos.EvalEl(2), Pos.EvalEl(3), 'r', 'filled', 'DisplayName', 'Elbow');
    
end