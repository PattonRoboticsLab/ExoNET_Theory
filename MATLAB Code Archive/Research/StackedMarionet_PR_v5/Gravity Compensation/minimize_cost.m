function  [optimal_param, average_err] = minimize_cost(p, options, mean_err_old)
    
    global desired_torque phi
    max_iter = 100;
    it = 1;
    toll = 1e-5;
    p_new = fminsearch('torque_cost', p, options);
    torque = stacks(p_new, phi);
    err = abs(desired_torque - torque);
    mean_err_new = mean(err);
    err_diff = abs(mean_err_new-mean_err_old);
    
    while (((err_diff > toll) || (mean_err_new > mean_err_old)) && (it < max_iter)) %when the difference between the new and the old costs decreases under a cetain threshold,
                                %we exit the while loop.  
        mean_err_old = mean_err_new;
        p_old = p_new;
        p_new = fminsearch('torque_cost', p_old, options);
        torque = stacks(p_new, phi);
        err = abs(desired_torque - torque);
        mean_err_new = mean(err);
        err_diff = abs(mean_err_new-mean_err_old);
        it = it+1;
    end
    
    optimal_param = p_new;
    average_err = mean_err_new;
end