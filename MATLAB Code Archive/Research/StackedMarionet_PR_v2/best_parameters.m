function [opt_param_err, opt_param_perc, min_mean, min_perc] = best_parameters(iter, options)
    
    global n desired_torque phi 
    mean_err = zeros(1, iter);    
    parameters = zeros(iter, 2*n);

    for i = 1:iter

        fprintf(' Evaluation number: %d \n', i);
        
        init_param = rand(1,(n*2)); %Initial guess for thetas and radii
        starting_torque = stacks(init_param, phi);
        mean_err_old = mean(abs(desired_torque - starting_torque));
        [optimal_param, average_err, average_perc] = minimize_cost(init_param, options, mean_err_old);
        parameters(i, :) = optimal_param;
        mean_err(i) = average_err;
        if i == 1
            min_mean = average_err;
            min_perc = average_perc;
        end

        if average_err <= min_mean
            min_mean = average_err;
            pos_mean = i;
        end
        if average_perc <= min_perc
            min_perc = average_perc;
            pos_perc = i;
        end
    end
    
    opt_param_err = parameters(pos_mean, :);
    opt_param_perc = parameters(pos_perc, :);
    
end


    