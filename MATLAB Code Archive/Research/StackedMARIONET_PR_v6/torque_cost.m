function cost=torque_cost(p)

global phi desired_torque n H X cost

tau_tot=stacks(p, phi);
cost=sum((tau_tot-desired_torque).^2);
scatter3(p(1),p(2),cost);
hold on




%% constraints
%radius=p(n+1:2*n);
low_val = 0.01;
high_val = 0.05;

radius = p(1, n+1:end);
angle = p(1, 1:n);

%angle = p(1);
%radius = p(2);
%     ang_low = 0;
%    ang_high = 2*pi;
    
for i=1:n   %Cost Punishment for negative radii
    
    if radius(i)<= low_val
        cost=cost+1e20*abs(radius(i))^3;
    end
    
    if radius(i) >= high_val     %Added condition on a maximum radius
        cost=cost+1e15*abs(radius(i)-high_val)^3;
    end
    
    %          if angle(i) <= ang_low
    %               cost = cost+1e20*abs(angle(i))^3;
    %           end
    % %
    %           if angle(i) >= ang_high
    %               cost=cost+1e15*abs(angle(i)-ang_high)^3;
    %           end
    
    %% update plot 
    %set(H, 'xdata',phi,  'ydata',tau_tot); drawnow; pause(.001);   
   
    
end
