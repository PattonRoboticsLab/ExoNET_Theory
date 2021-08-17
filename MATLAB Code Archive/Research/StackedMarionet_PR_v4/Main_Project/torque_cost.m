function cost=torque_cost(p)

    global phi desired_torque n desired_potential
    
    tau_tot=stacks(p, phi); 
    potential = trapz(phi,tau_tot);
    desired_potential = trapz(phi,desired_torque)
    
    cost=sum((potential-desired_potential).^2); %cost function with potentials
    
    
    %cost=sum((tau_tot-desired_torque).^2); %cost function with torques
    radius=p(n+1:2*n);
      
    low_val = 0.05;
    high_val = 1;
    for i=1:n   %Cost Punishment for negative radii
        
        if radius(i)<= low_val
            cost=cost+1e20*abs(radius(i))^3;
        end
        
        if radius(i) >= high_val     %Added condition on a maximum radius
            cost=cost+1e15*abs(radius(i)-high_val)^3;
        end
        
    end
    
    %for theta = -3:3
        %for radii = .05:.1
            %mesh(cost,theta,radii)
            %title('Cost vs Theta vs Radii')
       % end
  %  end
    
end
