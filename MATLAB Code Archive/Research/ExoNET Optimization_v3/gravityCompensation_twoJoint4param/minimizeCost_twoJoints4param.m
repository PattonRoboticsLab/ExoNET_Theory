function  [optimal_param, optimalTorque_sh, optimalTorque_el, averageErr_el, averageErr_sh, averagePerc_el, averagePerc_sh, Rsquared_el, Rsquared_sh]...
    = minimizeCost_twoJoints4param(p, options, mean_err_old)
    %04/16/2018
    global desiredTorque_el desiredTorque_sh phi_1 phi_2 n_sh
    max_iter = 50;
    it = 1;
    toll = 1e-6;
    p_new = fminsearch('torqueCost_twoJoint4param', p, options);
    [torque_sh, torque_el] = stacks_twoJoints4param(p_new, phi_1, phi_2);
    
    meanErr_el = zeros(n_sh, n_sh);
    percErr_el = zeros(n_sh, n_sh);
    meanErr_sh = zeros(n_sh, n_sh);
    percErr_sh = zeros(n_sh, n_sh);
    for i = 1:n_sh
        meanErr_el(:, i) = abs(desiredTorque_el(i, :) - torque_el(i, :));
        meanErr_sh(:, i) = abs(desiredTorque_sh(i, :) - torque_sh(i, :))';
        percErr_el(:, i) = abs(desiredTorque_el(i, :) - torque_el(i, :)./desiredTorque_el(i, :))*100';
        percErr_sh(:, i) = abs(desiredTorque_sh(i, :) - torque_sh(i, :)./desiredTorque_sh(i, :))*100';
    end
    mean_err_new = mean(mean(meanErr_el + meanErr_sh));
    err_diff = abs(mean_err_new-mean_err_old);
    
    while (((err_diff > toll) || (mean_err_new > mean_err_old)) && (it < max_iter)) %when the difference between the new and the old costs decreases under a cetain threshold,
                                %we exit the while loop.  
        mean_err_old = mean_err_new;
        p_old = p_new;
        p_new = fminsearch('torqueCost_twoJoint4param', p_old, options);
        
        %percErr = zeros(n_sh, n_sh);
        
        [torque_sh, torque_el] = stacks_twoJoints4param(p_new, phi_1, phi_2);
        meanErr_el = zeros(n_sh, n_sh);
        meanErr_sh = zeros(n_sh, n_sh);
        percErr_el = zeros(n_sh, n_sh);
        percErr_sh = zeros(n_sh, n_sh);

        for i = 1:n_sh
            meanErr_el(:, i) = abs(desiredTorque_el(i, :) - torque_el(i, :));
            meanErr_sh(:, i) = abs(desiredTorque_sh(i, :) - torque_sh(i, :))';
            percErr_el(:, i) = abs(desiredTorque_el(i, :) - torque_el(i, :)./desiredTorque_el(i, :))*100';
            percErr_sh(:, i) = abs(desiredTorque_sh(i, :) - torque_sh(i, :)./desiredTorque_sh(i, :))*100';
        end
        mean_err_new = mean(mean(meanErr_el + meanErr_sh));
        err_diff = abs(mean_err_new-mean_err_old);
        it = it+1;
    end
    Rsquared_el = zeros(1, n_sh);
    Rsquared_sh = zeros(1, n_sh);
    optimal_param = p_new;
    [optimalTorque_sh, optimalTorque_el] = stacks_twoJoints4param(optimal_param, phi_1, phi_2);
    for i = 1:n_sh
        SS_tot = sum((desiredTorque_el(i, :)-mean(desiredTorque_el(i, :))).^2);
        SS_res = sum((desiredTorque_el(i, :) - optimalTorque_el(i, :)).^2);
        Rsquared_el(i) = 1 - (SS_res/SS_tot);
        SS_tot = sum((desiredTorque_sh(i, :)-mean(desiredTorque_sh(i, :))).^2);
        SS_res = sum((desiredTorque_sh(i, :) - optimalTorque_sh(i, :)).^2);
        Rsquared_sh(i) = 1 - (SS_res/SS_tot);
    end
    averageErr_el = mean(meanErr_el);
    averageErr_sh = mean(meanErr_sh);
    averagePerc_el = mean(percErr_el);
    averagePerc_sh = mean(percErr_sh);
    
end