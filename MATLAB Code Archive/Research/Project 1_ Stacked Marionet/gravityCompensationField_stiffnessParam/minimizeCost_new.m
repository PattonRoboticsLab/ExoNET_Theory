function  [optimal_param, optimal_torque, average_err] = minimizeCost_new(p, options, mean_err_old)
    %04/16/2018
    global desired_torque phi n_sh
    max_iter = 100;
    it = 1;
    toll = 1e-5;
    p_new = fminsearch('torqueCost_new', p, options);
    torque = stacks(p_new, phi);
    meanErr = zeros(1, n_sh);
    for i = 1:n_sh
        err = abs(desired_torque(i, :) - torque');
        meanErr(i) = mean(err);
    end
    mean_err_new = sum(meanErr);
    err_diff = abs(mean_err_new-mean_err_old);
    
    while (((err_diff > toll) || (mean_err_new > mean_err_old)) && (it < max_iter)) %when the difference between the new and the old costs decreases under a cetain threshold,
                                %we exit the while loop.  
        mean_err_old = mean_err_new;
        p_old = p_new;
        p_new = fminsearch('torqueCost_new', p_old, options);
        torque = stacks(p_new, phi);
        meanErr = zeros(1, n_sh);
        for i = 1:n_sh
            err = abs(desired_torque(i, :) - torque');
            meanErr(i) = mean(err);
        end
        mean_err_new = sum(meanErr);
        err_diff = abs(mean_err_new-mean_err_old);
        it = it+1;
    end
    
    optimal_param = p_new;
    optimal_torque = stacks(optimal_param, phi);
    average_err = mean_err_new;
end