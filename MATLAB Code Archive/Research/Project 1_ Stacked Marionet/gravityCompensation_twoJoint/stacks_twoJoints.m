function [tauTot_sh, tauTot_el] = stacks_twoJoints(p, phi_1, phi_2)
    
    global spring_length momentArm_sh momentArm_el n_stackedMarionet L_springRest n_sh springStiffness n_el
    
    %k = springStiffness; %Spring Stiffness N/m
    L0 = L_springRest;
    n = n_stackedMarionet;
    %Units in N  
    tauTot_sh = zeros(n_sh, n_el);
    tauTot_el = zeros(n_el, n_sh);
    
    for j = 1:n_sh
        
        for i = 1:n
            
            phi =  phi_1(j)*ones(n_el, 1);   
            springLength = spring_length([p(i), p(i+n)], phi, phi_2);
            eps = ((springLength - L0));
            F_spring = springForce_calculationLinear(springStiffness, eps);
            tauTot_el(j, :) = tauTot_el(j, :) + (F_spring .* momentArm_el([p(i), p(i+n)], phi, phi_2))'; %torque matrix
            tauTot_sh(j, :) = tauTot_sh(j, :) + (F_spring .* momentArm_sh([p(i), p(i+n)], phi, phi_2))'; %torque matrix
            
        end
       
    end
    
    
    
end
        
