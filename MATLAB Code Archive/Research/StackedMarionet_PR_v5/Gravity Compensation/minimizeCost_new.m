function  [optimal_param, optimal_torque, average_err, averageErr_perc, Rsquared] = minimizeCost_new(p, options, mean_err_old)
    %04/16/2018
    global desired_torque phi n_sh
    max_iter = 50;
    it = 1;
    toll = 1e-6;
    p_new = fminsearch('torqueCost_new', p, options);
    torque = stacks(p_new, phi);
    meanErr = zeros(n_sh, n_sh);
    for i = 1:n_sh
        meanErr(:, i) = abs(desired_torque(i, :) - torque');
    end
    mean_err_new = mean(mean(meanErr));
    err_diff = abs(mean_err_new-mean_err_old);
    
    while (((err_diff > toll) || (mean_err_new > mean_err_old)) && (it < max_iter)) %when the difference between the new and the old costs decreases under a cetain threshold,
                                %we exit the while loop.  
        mean_err_old = mean_err_new;
        p_old = p_new;
        p_new = fminsearch('torqueCost_new', p_old, options);
        torque = stacks(p_new, phi);
        meanErr = zeros(n_sh, n_sh);
        percErr = zeros(n_sh, n_sh);
        for i = 1:n_sh
            meanErr(:, i) = abs(desired_torque(i, :) - torque');
            percErr(:, i) = meanErr(:, i)'./abs(desired_torque(i, :)).*100;
        end
        mean_err_new = mean(mean(meanErr));
        err_diff = abs(mean_err_new-mean_err_old);
        it = it+1;
    end
    optimal_param = p_new;
    optimal_torque = stacks(optimal_param, phi);
    Rsquared = zeros(1, n_sh);
    for i = 1:n_sh
        SS_tot = sum((desired_torque(i, :)-mean(desired_torque(i, :))).^2);
        SS_res = sum((desired_torque(i, :) - optimal_torque').^2);
        Rsquared(i) = 1 - (SS_res/SS_tot);
    end
    average_err = mean(meanErr);
    averageErr_perc = mean(percErr);
end