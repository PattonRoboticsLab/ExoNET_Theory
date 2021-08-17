function [tauTot_sh, tauTot_el] = stacks_twoJoints4param(p, phi_1, phi_2)
    
    global spring_length momentArm_sh momentArm_el n_stackedMarionet n_sh n_el
    
    n = n_stackedMarionet;
    %Units in N  
    tauTot_sh = zeros(n_sh, n_el);
    tauTot_el = zeros(n_el, n_sh);
    
    for j = 1:n_sh
        
        for i = 1:n
            
            phi =  phi_1(j)*ones(n_el, 1);   
            springLength = spring_length([p(i), p(i+n)], phi, phi_2);
            eps = ((springLength - p(i+2*n)));
            F_spring = springForce_calculationLinear(p(i+3*n), eps);
            tauTot_el(j, :) = tauTot_el(j, :) + (F_spring .* momentArm_el([p(i), p(i+n)], phi, phi_2))'; %torque matrix
            tauTot_sh(j, :) = tauTot_sh(j, :) + (F_spring .* momentArm_sh([p(i), p(i+n)], phi, phi_2))'; %torque matrix
            
        end
       
    end
    
    
    
end
        
