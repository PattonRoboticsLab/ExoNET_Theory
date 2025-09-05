function [p_opt, cost_opt] = run_single_optimization(cost_func, p0, A, b, Aeq, beq, lb, ub, nonlcon, options)
% Single optimization run with error handling
try
    [p_opt, cost_opt] = fmincon(cost_func, p0, A, b, Aeq, beq, lb, ub, nonlcon, options);
    
    % Validate result
    if ~isfinite(cost_opt) || any(~isfinite(p_opt))
        cost_opt = inf;
        p_opt = p0; % Return initial guess if optimization failed
    end
    
catch ME
    warning('Optimization failed: %s', ME.message);
    p_opt = p0;
    cost_opt = inf;
end
end