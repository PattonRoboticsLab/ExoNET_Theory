function cost = torqueCost_4param(p)
 
    global phi desired_torque n
    
    tau_tot = stacks_4param(p, phi); 
    cost = sum((tau_tot-desired_torque).^2);
    radius = p(n+1:2*n);
    springRest = p(2*n+1:3*n
    springStiff = p(3*n+1:4*n);
    radiusLow = 0.05;
    radiusHigh = 0.1;
    restLow = .05;
    restHigh = .55;
    stiffLow = 50;
    stiffHigh = 2000;
    
    for i=1:n   
        
        if radius(i)<= radiusLow
            cost=cost+1e20*abs(radius(i)-radiusLow)^3;
        end
        
        if radius(i) >= radiusHigh     
            cost=cost+1e15*abs(radius(i)-radiusHigh)^3;
        end
       
        if springRest(i)<= restLow
            cost=cost+1e20*abs(springRest(i)-restLow)^3;
        end
        
        if springRest(i) >= restHigh     
            cost=cost+1e15*abs(springRest(i)-restHigh)^3;
        end
        
        if springStiff(i)<= stiffLow
            cost=cost+1e20*abs(springStiff(i)-stiffLow)^3;
        end
        
        if springStiff(i) >= stiffHigh     
            cost=cost+1e15*abs(springStiff(i)-stiffHigh)^3;
        end
       
    end
  
end
