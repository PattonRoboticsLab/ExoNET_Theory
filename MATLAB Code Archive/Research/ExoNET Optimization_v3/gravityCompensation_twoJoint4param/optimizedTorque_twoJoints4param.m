function [bestParam, averageErr_el, averageErr_sh, averagePerc_el, averagePerc_sh, Rsquared_el, Rsquared_sh] =...
    optimizedTorque_twoJoints4param(Lengths, Weights, shoulder_angle, elbow_angle)
    
    global phi_1 phi_2 desiredTorque_el desiredTorque_sh n_stackedMarionet momentArm_sh momentArm_el spring_length L_ForeArm L_UpperArm n_sh n_el
    
    n_sh = length(shoulder_angle);
    n_el = length(elbow_angle);
    upperArm_weight = Weights(1);
    foreArm_weight = Weights(2);
    hand_weight = Weights(3);
    
    L_UpperArm = Lengths(1);
    L_ForeArm = Lengths(2);
    
    options.MaxIter = Inf;
    options.MaxFunEvals = Inf;
    init_param = rand(1,(n_stackedMarionet*4));
    desiredMatrix_el = zeros(n_sh, n_el);
    desiredMatrix_sh = zeros(n_sh, n_el);
    forceMatrix_general = zeros(2*(n_sh), n_el*2);
    
    for i = 1:n_sh
               
        alpha = shoulder_angle(i)+elbow_angle;

        desiredMatrix_el(i, :) = (foreArm_weight/2+hand_weight)*L_ForeArm.*cos(alpha);
    end
    
    for i = 1:n_el
        desiredMatrix_sh(i, :) = (upperArm_weight/2+foreArm_weight+hand_weight)*L_UpperArm*cos(shoulder_angle')+desiredMatrix_el(:, i);
    end
    
    %desiredMatrix_tot = desiredMatrix_sh + desiredMatrix_el;
    
    %optimization elbow torque
    beta = betaCalculation(Lengths, elbow_angle, n_el);
    phi_2 = elbow_angle';
    phi_1 = shoulder_angle';
    spring_length = @(p, phi_1, phi_2) sqrt((L_ForeArm*sin(phi_2)./sin(beta)).^2+p(2)^2 -...
        (L_ForeArm*sin(phi_2)./sin(beta))*p(2).*cos(p(1)-beta-phi_1));
    
    momentArm_sh = @(p, phi_1, phi_2)   (L_ForeArm*sin(phi_2)./sin(beta))*p(2).*sin(p(1)-beta-phi_1)./...
        sqrt((L_ForeArm*sin(phi_2)./sin(beta)).^2+p(2)^2 -...
        (L_ForeArm*sin(phi_2)./sin(beta))*p(2).*cos(p(1)-beta-phi_1));%arm for the force
    
    delta = asin(L_UpperArm.*sin(beta)./L_ForeArm);
    
    momentArm_el = @(p, phi_1, phi_2) L_ForeArm.*sin(delta + asin(p(2).*sin(p(1)-beta-phi_1)./sqrt((L_ForeArm*sin(phi_2)./...
        sin(beta)).^2+p(2)^2 - (L_ForeArm*sin(phi_2)./sin(beta))*p(2).*cos(p(1)-beta-phi_1))));
    
    [startingTorque_sh, startingTorque_el] = stacks_twoJoints4param(init_param, phi_1, phi_2); %torque with random guess (matrix!)
    meanErr_el = zeros(n_el, n_sh);
    meanErr_sh = zeros(n_sh, n_el);
    for i = 1:n_sh
        meanErr_el(:, i) = abs(desiredMatrix_el(i, :) - startingTorque_el(i, :));
        meanErr_sh(:, i) = abs(desiredMatrix_sh(i, :) - startingTorque_sh(i, :))';
    end
    meanErr_tot = mean(mean(meanErr_el + meanErr_sh));
    desiredTorque_el = desiredMatrix_el;
    desiredTorque_sh = desiredMatrix_sh;
    [bestParam, optimalTorque_sh, optimalTorque_el, averageErr_el, averageErr_sh, averagePerc_el, averagePerc_sh, Rsquared_el, Rsquared_sh] =...
        minimizeCost_twoJoints4param(init_param, options, meanErr_tot);
    figure(2)
    for i = 1:n_sh
        phi =  phi_1(i)*ones(n_el, 1);
        plot3(phi, phi_2, optimalTorque_el(i, :), 'b')
        hold on
        plot3(phi, phi_2, desiredMatrix_el(i, :), 'r');  
        hold on
        plot3(phi, phi_2, optimalTorque_sh(i, :), 'g')
        hold on
        plot3(phi, phi_2, desiredMatrix_sh(i, :), 'k');  
        hold on
    end
    xlabel('\phi_1')
    ylabel('\phi_2')
    zlabel('Torque')
    grid on
    
    figure()
    plot(phi_1, desiredMatrix_sh(n_sh/2, :), 'r')
    xlabel('\phi_1 [rad]')
    ylabel('Torque [Nm]')
    title('Shoulder Torque VS Shoulder Angle')

    grid on
    figure()
    plot(phi_2, desiredMatrix_el(n_sh/2, :), 'r')
    xlabel('\phi_2 [rad]')
    ylabel('Torque [Nm]')
    grid on
    title('Elbow Torque VS Elbow Angle')

    ct_i = 0;
    maxForce_overallVec = zeros(1, n_sh);
    for i = 1:n_sh
        
        ct_j = 0;
        
        for j = 1:n_el
            
            % Optimum forces
            angles = [shoulder_angle(i); elbow_angle(j)];
            torqueVector_optim = [optimalTorque_sh(i, j); optimalTorque_el(i, j)];
            jacobianMatrix_optim = jacobian(angles, Lengths);
            
            if (cond(jacobianMatrix_optim) > 1e10)
                forceVector_optim = [0; 0];
            else
                forceVector_optim = ((jacobianMatrix_optim)')\torqueVector_optim;
            end
            
            % Desired forces
            torqueVector_desired = [desiredMatrix_sh(i, j); desiredMatrix_el(i, j)];
            jacobianMatrix_desired = jacobian(angles, Lengths);
            
            if (cond(jacobianMatrix_desired) > 1e10)
                forceVector_desired = [0; 0];
            else
                forceVector_desired = ((jacobianMatrix_desired)')\torqueVector_desired;
            end
            
            % Finding max force
            
            forceMatrix = [forceVector_optim, forceVector_desired];
            max_force = max(max(abs(forceMatrix)));
            forceMatrix_general(ct_i+i:ct_i+i+1, ct_j+j:ct_j+j+1) = forceMatrix;
            
            if (j == 1)
                maxForce_overallVec(i) = max_force;
            elseif (j >= 1 && max_force > maxForce_overallVec(i) && max_force ~= Inf)
                maxForce_overallVec(i) = max_force;
            end
            
            ct_j = ct_j+1;
            
        end
        
        ct_i = ct_i+1;
        
    end
    
    ct_i = 0;
    maxForce_overall = max(maxForce_overallVec);
    for i = 1:n_sh
        
        ct_j = 0;
        R_el = [cos(shoulder_angle(i)), -sin(shoulder_angle(i)); sin(shoulder_angle(i)), cos(shoulder_angle(i))];
        sh_pos = [0; 0];
        el_pos = sh_pos+(R_el*[L_UpperArm; 0]);
        
        for j = 1:n_el
            
            alpha = shoulder_angle(i)+elbow_angle;
            R_wr = [cos(alpha(j)), -sin(alpha(j)); sin(alpha(j)), cos(alpha(j))];
            wrist_position = el_pos+(R_wr*[L_ForeArm; 0]);
            
            forceOptim_x = forceMatrix_general(ct_i+(i), ct_j+(j));
            forceOptim_y = forceMatrix_general(ct_i+(i+1), ct_j+(j));
            forceDesired_x = forceMatrix_general(ct_i+(i), ct_j+(j+1));
            forceDesired_y = forceMatrix_general(ct_i+(i+1), ct_j+(j+1));
            
            xForce_optNorm = (forceOptim_x/maxForce_overall)*.15;
            yForce_optNorm = (forceOptim_y/maxForce_overall)*.15;
            xForce_desNorm = (forceDesired_x/maxForce_overall)*.15;
            yForce_desNorm = (forceDesired_y/maxForce_overall)*.15;
            
            
            if ((xForce_optNorm == 0 && yForce_optNorm == 0) || (xForce_desNorm == 0 && yForce_desNorm == 0))
                fprintf('Arrow is NULL beacuse of Jacobian singularity \n');
            else
                arrow(wrist_position, wrist_position+[xForce_desNorm; yForce_desNorm], 'Color', 'r', 'Linewidth', 4,...
                    'Length', 3);
                hold on
                arrow(wrist_position, wrist_position+[xForce_optNorm; yForce_optNorm], 'Color', 'k', 'Linewidth', 2,...
                    'Length', 3);
                hold on
            end
            
            ct_j = ct_j+1;
            
        end
        
        ct_i = ct_i+1;
        
    end
    
    axis off
    
    figure(2)
    [maxVal_el, maxIndex_el] = min(Rsquared_el);
    plot(phi_2, desiredMatrix_el(maxIndex_el, :), 'r')
    hold on
    plot(phi_2, optimalTorque_el(maxIndex_el, :), 'b')
    title('Worst Approximation for Elbow Torque')
    grid on
    xlabel('\phi_2 [rad]')
    ylabel('Torque [Nm]')

    
    figure(3)
    [minVal_el, minIndex_el] = max(Rsquared_el);
    plot(phi_2, desiredMatrix_el(minIndex_el, :), 'r')
    hold on
    plot(phi_2, optimalTorque_el(minIndex_el, :), 'b')
    title('Best Approximation for Elbow Torque')
    grid on
    xlabel('\phi_2 [rad]')
    ylabel('Torque [Nm]')

    
    figure(4)
    [maxVal_sh, maxIndex_sh] = min(Rsquared_sh);
    plot(phi_2, desiredMatrix_sh(maxIndex_sh, :), 'r')
    hold on
    plot(phi_2, optimalTorque_sh(maxIndex_sh, :), 'b')
    title('Worst Approximation for Shoulder Torque')
    grid on
    xlabel('\phi_2 [rad]')
    ylabel('Torque [Nm]')

    figure(5)
    [minVal_sh, minIndex_sh] = max(Rsquared_sh);
    plot(phi_2, desiredMatrix_sh(minIndex_sh, :), 'r')
    hold on
    plot(phi_2, optimalTorque_sh(minIndex_sh, :), 'b')
    title('Best Approximation for Shoulder Torque')
    grid on
    xlabel('\phi_2 [rad]')
    ylabel('Torque [Nm]')

end 
