function tau_tot = stacks(p, phi)
    
    global spring y n_stackedMarionet L_springRest n_sh springStiffness 
    
    %k = springStiffness; %Spring Stiffness N/m
    L0 = L_springRest;
    n = n_stackedMarionet;
    %Units in N  
    tau_tot = zeros(n_sh, 1);
    
    for i = 1:n
       
       spring_length = spring([p(i), p(i+n)], phi);
       eps = ((spring_length - L0));
       F_spring = springForce_calculationLinear(springStiffness, eps);
       tau_tot = tau_tot + F_spring .* y([p(i), p(i+n)], phi);
       
    end
    
end
        
