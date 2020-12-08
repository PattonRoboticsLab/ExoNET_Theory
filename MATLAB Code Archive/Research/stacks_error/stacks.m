function tau_tot=stacks(p,phi)

    global stack
    
    k=150; %Spring Stiffness N/m
    x=.3; %displ of spring in m
    F_spring=k*x;      %Units in N  
    
    tau_tot = F_spring .* stack(p, phi);

end
        
