function spring_force = springForce_calculationLinear(stiffness, displacement)
    
    x = length(displacement);
    spring_force = zeros(x, 1);
    
    for i = 1:x
       spring_force(i) = -stiffness*displacement(i); 
    end

end