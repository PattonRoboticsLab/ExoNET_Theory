function cost=torque_cost(p)

    global phi desired_torque n
    
    tau_tot=stacks(p, phi); 
    cost=sum((tau_tot-desired_torque).^2);
    %radius = p(2)
    %angle = p(1)
    %plot3(radius,angle,cost)
    radius=p(n+1:2*n);
    
    
    %radius = p(2);
    %angle = p(1);
    
    ang_low = 0.01;
    ang_high = 2.5;
      
    low_val = 0.001;
    high_val = .05;
    for i=1:n   %Cost Punishment for negative radii
        
        if radius(i)<= low_val
            cost=cost+1e20*abs(radius(i))^3;
        end
        
        if radius(i) >= high_val     %Added condition on a maximum radius
            cost=cost+1e15*abs(radius(i)-high_val)^3;
        end
        
        if angle(i) <= ang_low
            cost = cost+1e20*abs(angle(i))^3;
        end
        
        if angle(i) >= ang_high
            cost=cost+1e15*abs(angle(i)-ang_high)^3;
        end
    end
    
    %for theta = -3:3
     %   for radii = .05:.1
      %      mesh(cost,theta,radii)
       %     title('Cost vs Theta vs Radii')
       % end
    %end
    
end
