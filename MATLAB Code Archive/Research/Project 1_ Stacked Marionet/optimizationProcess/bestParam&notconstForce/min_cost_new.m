function  [mean_err_new, p_new] = min_cost_new(p, options, mean_err_old)
    
    global tau_d phi

    toll = 1e-5;
    p_new = fminsearch('torque_cost', p, options);
    torque = stacks(p_new, phi);
    err = abs(tau_d - torque);
    mean_err_new = mean(err);
    err_diff = abs(mean_err_new-mean_err_old);
    
    while ((err_diff > toll) || (mean_err_new > mean_err_old)) %when the difference between the new and the old costs 
                                %decreases under a cetain threshold, we exit the while loop.  
        mean_err_old = mean_err_new;
        p_old = p_new;
        p_new = fminsearch('torque_cost', p_old, options);
        torque = stacks(p_new, phi);
        err = abs(tau_d - torque);
        mean_err_new = mean(err);
        err_diff = abs(mean_err_new-mean_err_old);
        
    end
    
end