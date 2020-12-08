function cost=torque_cost(p)

    global phi tau_d n
    
    tau_tot=stacks(p, phi); 
    cost=sum((tau_tot-tau_d).^2);
    radius=p(n+1:2*n);
    
    for i=1:n   %Cost Punishment for negative radii
        
        if radius(i)<=0 
            cost=cost+1e10*radius(i)^2;
        end
        
        if radius(i) >= 0.2     %Added condition on a maximum radius
            cost=cost+1e20*radius(i)^2;
        end
        
    end
  
end
