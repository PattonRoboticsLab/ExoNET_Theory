function spring_force = springForce_calculationLinear(stiffness, displacement)
    
    x = length(displacement);
    spring_force = zeros(x, 1);
    
    for i = 1:x
       spring_force(i) = -stiffness*displacement(i); 
%         if (deformation(i) < 0)
%             spring_force(i) = 0;
%         else
%             spring_force(i) = -stiffness*deformation(i);
%         end
    
    end

end