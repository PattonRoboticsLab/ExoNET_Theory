function cost = torqueCost_twoJoint4param(p)

    global phi_1 phi_2 desiredTorque_el desiredTorque_sh n_stackedMarionet n_sh
    
    costVector_el = zeros(1, n_sh);
    costVector_sh = zeros(1, n_sh);

    n = n_stackedMarionet;
    radius = p(n+1:2*n);
    springRest = p(2*n+1:3*n); 
    springStiff = p(3*n+1:4*n);
    radiusLow = 0.05;
    radiusHigh = 0.1;
    restLow = .05;
    restHigh = .55;
    stiffLow = 50;
    stiffHigh = 2000;

    
    for i = 1:n_sh
        
        [tauTot_sh, tauTot_el] = stacks_twoJoints4param(p, phi_1, phi_2); 
        costVector_el(i) = sum((tauTot_el(i, :)-desiredTorque_el(i, :)).^2);
        costVector_sh(i) = sum((tauTot_sh(i, :)-desiredTorque_sh(i, :)).^2);
    end
    
    cost = sum(costVector_el+costVector_sh);
    
    for j = 1:n   %Cost Punishment for negative radii

        if radius(j)<= radiusLow
            cost = cost+1e20*abs(radius(j)-radiusLow)^3;
        end

        if radius(j) >= radiusHigh     %Added condition on a maximum radius
            cost = cost+1e15*abs(radius(j)-radiusHigh)^3;
        end
        
        if springRest(j)<= restLow
            cost=cost+1e20*abs(springRest(j)-restLow)^3;
        end
        
        if springRest(j) >= restHigh     
            cost=cost+1e15*abs(springRest(j)-restHigh)^3;
        end
        
        if springStiff(j)<= stiffLow
            cost=cost+1e20*abs(springStiff(j)-stiffLow)^3;
        end
        
        if springStiff(j) >= stiffHigh     
            cost=cost+1e15*abs(springStiff(j)-stiffHigh)^3;
        end

    end
    
end