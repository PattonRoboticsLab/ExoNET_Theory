function tau_tot=stacks(p, phi)
    
    global spring y n L_springRest %springStiffness 
    
    %k = springStiffness; %Spring Stiffness N/m
    L0 = L_springRest;
    
    %Units in N 
    tau_tot = zeros(length(phi), 1);
    
    for i = 1:n
       
       spring_length = spring([p(i), p(i+n)], phi);
       eps = ((spring_length - L0));
       F_spring = springForce_calculationLinear(p(i+2*n),eps);
       tau_tot = tau_tot + F_spring .* y([p(i), p(i+n)], phi);
       
    end
    
end
        
