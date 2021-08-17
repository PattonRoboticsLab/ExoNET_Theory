function spring_force = springForce_calculationLinear(stiffness,displacement)
%Function objective: Calculate the Force of the Spring based on it's
%stiffness value and displacement based on Hooke's Law: F = -k*dx

x = length(displacement);
spring_force = zeros(x,1);

for i = 1:x
    spring_force(i) = -stiffness*displacement;
    %if (deformation(i)<0)
        %spring_force(i) = 0
    %else
        %spring_force(i) = -stiffness*deformation(i);
    %end
end
end
