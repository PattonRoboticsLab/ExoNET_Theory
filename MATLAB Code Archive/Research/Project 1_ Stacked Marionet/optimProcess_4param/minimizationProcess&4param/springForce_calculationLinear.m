function spring_force = springForce_calculationLinear(stiffness, deformation)
    
    x = length(deformation);
    spring_force = zeros(x, 1);
    
    for i = 1:x
        
        spring_force(i) = -stiffness*deformation(i);
            
    end

end