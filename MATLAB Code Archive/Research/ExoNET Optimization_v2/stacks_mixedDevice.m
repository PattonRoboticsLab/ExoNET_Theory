function [tauTot_sh, tauTot_el] = stacks_mixedDevice(p, phi_1, phi_2)
    
    global spring_length momentArm_sh momentArm_el nTwo_stackedMarionet L_springRest n_sh springStiffness n_el...
        momentArm_singleEl springLength_singleEl momentArm_singleSh springLength_singleSh nSh_stackedMarionet...
        nEl_stackedMarionet
    
    %k = springStiffness; %Spring Stiffness N/m
    L0 = L_springRest;
    %Units in N  
    tauTot_sh = zeros(n_sh, n_el);
    tauTot_el = zeros(n_el, n_sh);
    
    for j = 1:n_sh
        
        for i = 1:nTwo_stackedMarionet
            
            phi =  phi_1(j)*ones(n_el, 1);   
            springLength = spring_length([p(i), p(i+nTwo_stackedMarionet)], phi, phi_2);
            eps_twoJoint = ((springLength - L0));
            F_springTwo = springForce_calculationLinear(springStiffness, eps_twoJoint);
            tauTot_el(j, :) = tauTot_el(j, :) +...
                (F_springTwo .* momentArm_el([p(i), p(i+nTwo_stackedMarionet)], phi, phi_2))'; %torque matrix
            tauTot_sh(j, :) = tauTot_sh(j, :) +...
                (F_springTwo .* momentArm_sh([p(i), p(i+nTwo_stackedMarionet)], phi, phi_2))'; %torque matrix
            
        end
        
        for i = 1:nEl_stackedMarionet
            
            springLength_el = springLength_singleEl([p(i+2*nTwo_stackedMarionet),...
                p(i+2*nTwo_stackedMarionet+nEl_stackedMarionet)], phi); 
            eps_singleEl = springLength_el - L0;
            F_springEl = springForce_calculationLinear(springStiffness, eps_singleEl);
            tauTot_el(j, :) = tauTot_el(j, :) +...
                (F_springEl.*momentArm_singleEl([p(i+2*nTwo_stackedMarionet),...
                p(i+2*nTwo_stackedMarionet+nEl_stackedMarionet)], phi_2))'; %torque matrix
                        
        end
        
        for i = 1:nSh_stackedMarionet
            
            springLength_sh = springLength_singleSh([p(i+2*nTwo_stackedMarionet+2*nEl_stackedMarionet),...
                p(i+2*nTwo_stackedMarionet+2*nEl_stackedMarionet+nSh_stackedMarionet)], phi_2);
            eps_singleSh = springLength_sh - L0;
            F_springSh = springForce_calculationLinear(springStiffness, eps_singleSh);
            tauTot_sh(j, :) = tauTot_sh(j, :) +...
                 (F_springSh.*momentArm_singleSh([p(i+2*nTwo_stackedMarionet+2*nEl_stackedMarionet),...
                p(i+2*nTwo_stackedMarionet+2*nEl_stackedMarionet+nSh_stackedMarionet)], phi))'; %torque matrix
            
        end
    end
        
end
        
