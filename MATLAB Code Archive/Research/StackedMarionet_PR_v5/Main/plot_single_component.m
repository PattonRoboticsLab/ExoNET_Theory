function plot_single_component(param) 
    
    global phi n L spring springStiffness L_springRest 
    k=springStiffness; %Spring Stiffness N/m
    
    for i = 1:n
        spring_length = spring([param(i), param(i+n)], phi);
        L0 = L_springRest;
        eps = ((spring_length- L0));
        F_spring = springForce_calculationLinear(k, eps);
        y = @(p, phi) F_spring.*(L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)));
        hold on
        t = y([param(i) param(i+n)], phi);
        plot(phi, t, 'g');
        text(phi(1)-.48, t(end-3), ['Marionet', num2str(i)]);
        
    end
    
end
