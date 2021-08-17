function cost = torque_cost(p)

global phi desired_torque n

tau_tot=stack(p,phi);
cost = sum((tau_tot - desired_torque).^2);
radius = p(n+1:2*n);
low_val = 0.05;
high_val = 0.1;

for i = 1:n
    if radius(i) <= low_val
        cost = cost + 1e20*abs(radius(i))^3;
    end
    
    if radius(i) >= high_val % added condition on a maximum radius
        cost = cost+ 1e15*abs(radius(i)-high_val)^3;
    end
end
end
