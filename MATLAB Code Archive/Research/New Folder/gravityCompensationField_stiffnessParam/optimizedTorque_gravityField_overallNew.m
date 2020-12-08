function [bestParam_sh, bestParam_el] = optimizedTorque_gravityField_overallNew(Lengths, Weights, shoulder_angle, elbow_angle)
    
    global phi desired_torque n_stackedMarionet y spring L n_sh
    
    n_sh = length(shoulder_angle);
    n_el = length(elbow_angle);
    upperArm_weight = Weights(1);
    foreArm_weight = Weights(2);
    hand_weight = Weights(3);
    
    L_UpperArm = Lengths(1);
    L_ForeArm = Lengths(2);
    
    options.MaxIter = Inf;
    options.MaxFunEvals = Inf;
    init_param = rand(1,(n_stackedMarionet*3));
    paramMatrix_el = zeros(1, n_stackedMarionet*3);
    paramMatrix_sh = zeros(1, n_stackedMarionet*3);
    desiredMatrix_el = zeros(n_sh, n_el);
    desiredMatrix_sh = zeros(n_el, n_sh);
    forceMatrix_general = zeros(2*(n_sh), n_el*2);
    
    for i = 1:n_sh
               
        alpha = shoulder_angle(i)+elbow_angle;

        desiredMatrix_el(i, :) = (foreArm_weight/2+hand_weight)*L_ForeArm.*cos(alpha);
        desiredMatrix_sh(i, :) = (upperArm_weight/2+foreArm_weight+hand_weight)*L_UpperArm*cos(shoulder_angle(i))+desiredMatrix_el(:, 1);
        
    end
    
    %optimization elbow torque
    L = L_ForeArm;
    phi = elbow_angle';
    y = @(p, phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); %arm for the force 
    spring = @(p, phi) sqrt(L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi)); %spring length
    startingTorque_el = stacks(init_param, phi); %torque with random guess
    meanErr = zeros(1, n_sh);
    for i = 1:n_sh
        err = abs(desiredMatrix_el(i, :) - startingTorque_el');
        meanErr(i) = mean(err);
    end
    meanErr_el = sum(meanErr);
    desired_torque = desiredMatrix_el;
    [bestParam_el, optimalTorque_el] = minimizeCost_new(init_param, options, meanErr_el);


    %optimization shoulder torque
    L = L_UpperArm;
    phi = shoulder_angle';
    y = @(p, phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); %arm for the force 
    spring = @(p, phi) sqrt(L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi)); %spring length
    startingTorque_sh = stacks(init_param, phi); %torque with random guess
    meanErr = zeros(1, n_sh);
    for i = 1:n_sh
        err = abs(desiredMatrix_sh(i, :) - startingTorque_sh');
        meanErr(i) = mean(err);
    end
    meanErr_sh = sum(meanErr);
    desired_torque = desiredMatrix_sh;
    [bestParam_sh, optimalTorque_sh] = minimizeCost_new(init_param, options, meanErr_sh);

    ct_i = 0;
    maxForce_overallVec = zeros(1, n_sh);
    for i = 1:n_sh
        
        ct_j = 0;
        
        for j = 1:n_el
            
            % Optimum forces
            angles = [shoulder_angle(i); elbow_angle(j)];
            torqueVector_optim = [optimalTorque_sh(j); optimalTorque_el(j)];
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

end

