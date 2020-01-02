function tau_tot=stacks_4param(p, phi)
    
    global spring y n    

    tau_tot = zeros(length(phi), 1);
    
    for i = 1:n
       
       spring_length = spring([p(i), p(i+n)], phi);
       eps = ((spring_length - p(i+2*n)));
       F_spring = springForce_calculationLinear(p(i+3*n), eps);
       tau_tot = tau_tot + F_spring .* y([p(i), p(i+n)], phi);
       
    end
    
end
        
