function plot_single_component(param) 
    
    global phi n L spring
        
    for i = 1:n
        spring_length = spring([param(i), param(i+n)], phi);
        eps = ((spring_length - param(i+2*n)));
        
        F_spring = springForce_calculationLinear(param(i+3*n), eps);
        y = @(p, phi) F_spring.*(L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)));
        hold on
        plot(phi, y([param(i) param(i+n)], phi), 'c');
    end
end
