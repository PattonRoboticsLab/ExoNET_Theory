function create_stack

    global n stack d_stack L
    
    
    for i = 1:n
       
        y = @(p, phi) (L*p(i+n)*sin(p(i)-phi))./sqrt(p(i+n)^2+L^2-(2*p(i+n)*L*cos(p(i)-phi)));
        
        dy = @(p, phi) L*p(i+n).*(L.*cos(phi-p(i))-p(i+n)).*(p(i+n).*cos(phi-p(i))-L)./...
            (L^2-2*p(i+n)*L.*cos(phi-p(i))+p(i+n)^2).^(3/2);
        
        if i == 1
            
            stack = @(p, phi) y(p, phi);
            d_stack = @(p, phi) dy(p, phi);
            
        else
            
        stack = @(p, phi) stack(p, phi) + y(p, phi);
        d_stack = @(p, phi) d_stack(p, phi) + dy(p, phi);
        
        end
    end
        
end