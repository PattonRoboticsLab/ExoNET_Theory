function cost = torqueCost_new(p)

    global phi_1 phi_2 desiredTorque_el desiredTorque_sh n_stackedMarionet n_sh
    
    costVector_el = zeros(1, n_sh);
    costVector_sh = zeros(1, n_sh);

    n = n_stackedMarionet;
    radius = p(n+1:2*n);
%     stiffness = p(2*n+1:3*n); 
    low_val = 0.05;
    high_val = 0.1;
%     k_low = 50;
%     k_high = 1000;
    
    for i = 1:n_sh
        
        [tauTot_sh, tauTot_el] = stacks_twoJoints(p, phi_1, phi_2); 
        costVector_el(i) = sum((tauTot_el(i, :)-desiredTorque_el(i, :)).^2);
        costVector_sh(i) = sum((tauTot_sh(i, :)-desiredTorque_sh(i, :)).^2);
    end
    
    cost = sum(costVector_el+costVector_sh);
    
    for j = 1:n   %Cost Punishment for negative radii

        if radius(j)<= low_val
            cost = cost+1e20*abs(radius(j))^3;
        end

        if radius(j) >= high_val     %Added condition on a maximum radius
            cost = cost+1e15*abs(radius(j)-high_val)^3;
        end

%         if stiffness(j) <= k_low     %Added condition on a minimum stiffness
%             cost = cost+1e15*abs(stiffness(j)-k_low)^3;
%         end
% 
%         if stiffness(j) >= k_high     %Added condition on a maximum stiffness
%             cost = cost+1e15*abs(stiffness(j)-k_high)^3;
%         end

    end
    
end