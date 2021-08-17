function spring_force = springForce_calculationTraction(stiffness, deformation)
    
    x = length(deformation);
    spring_force = zeros(x, 1);
    
    for i = 1:x
        
        if (deformation(i) < 0)
            spring_force(i) = 0;
        else
            spring_force(i) = -stiffness*deformation(i);
        end
    
    end
 
end
