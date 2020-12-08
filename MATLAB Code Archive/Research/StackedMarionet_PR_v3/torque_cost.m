function cost = torque_cost(p)

global phi desired_torque n

torque_total = stack(p,phi);
cost = sum((torque_total - desired_torque).^2);
radius = p(n+1:2*n);

low_value = 0.05;
high_value = 0.1;

for i = 1:n %cost punishment for negative radii
    if radius(i) <= low_value
        cost = cost + 1e20 * abs(radius(i))^3;
    end
    
    if radius(i) >= high_value
        cost = cost + 1e15*abs(radius(i)-high_value)^3;
    end
end
end
