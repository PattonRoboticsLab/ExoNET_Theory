function cost = torqueCost_mixedDevice(p)

    global phi_1 phi_2 desiredTorque_el desiredTorque_sh nTwo_stackedMarionet nEl_stackedMarionet nSh_stackedMarionet n_sh
    
    costVector_el = zeros(1, n_sh);
    costVector_sh = zeros(1, n_sh);

    radiusTwo = p(nTwo_stackedMarionet+1:2*nTwo_stackedMarionet);
    radiusEl = p(2*nTwo_stackedMarionet+nEl_stackedMarionet+1:2*nTwo_stackedMarionet+2*nEl_stackedMarionet);
    radiusSh = p(2*nTwo_stackedMarionet+2*nEl_stackedMarionet+1:2*nTwo_stackedMarionet+2*nEl_stackedMarionet+2*nSh_stackedMarionet);
    
    low_val = 0.05;
    high_val = 0.1;
    
    for i = 1:n_sh
        
        [tauTot_sh, tauTot_el] = stacks_mixedDevice(p, phi_1, phi_2); 
        costVector_el(i) = sum((tauTot_el(i, :)-desiredTorque_el(i, :)).^2);
        costVector_sh(i) = sum((tauTot_sh(i, :)-desiredTorque_sh(i, :)).^2);
    end
    
    cost = sum(costVector_el+costVector_sh);
    
    for j = 1:nTwo_stackedMarionet   %Cost Punishment for negative radii

        if radiusTwo(j)<= low_val
            cost = cost+1e20*abs(radiusTwo(j))^3;
        end

        if radiusTwo(j) >= high_val     %Added condition on a maximum radius
            cost = cost+1e15*abs(radiusTwo(j)-high_val)^3;
        end

    end
    
    for j = 1:nEl_stackedMarionet   %Cost Punishment for negative radii

        if radiusEl(j)<= low_val
            cost = cost+1e20*abs(radiusEl(j))^3;
        end

        if radiusEl(j) >= high_val     %Added condition on a maximum radius
            cost = cost+1e15*abs(radiusEl(j)-high_val)^3;
        end

    end
    
    for j = 1:nSh_stackedMarionet   %Cost Punishment for negative radii

        if radiusSh(j)<= low_val
            cost = cost+1e20*abs(radiusSh(j))^3;
        end

        if radiusSh(j) >= high_val     %Added condition on a maximum radius
            cost = cost+1e15*abs(radiusSh(j)-high_val)^3;
        end

    end
    
end