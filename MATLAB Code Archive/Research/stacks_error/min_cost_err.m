function  p_new = min_cost_err(p, options)
    
    toll = 1e-5;
    p_new = fminsearch('torque_cost', p, options);
    old_cost = torque_cost(p);
    new_cost = torque_cost(p_new);
    cost_diff = abs(old_cost-new_cost);
    
    while (cost_diff > toll) %when the difference between the new and the old costs decreases under a cetain threshold,
                                %we exit the while loop.  
       p_old = p_new;
       p_new = fminsearch('torque_cost', p_old, options);
       old_cost = new_cost;
       new_cost = torque_cost(p_new);
       cost_diff = abs(old_cost-new_cost);

    end
    
end