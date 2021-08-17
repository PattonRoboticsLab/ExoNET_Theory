function torque_total = stacks(p,phi)

global Lsp mom_arm n k_sp Lsp_Rest

k = k_sp;
L0 = Lsp_Rest;

%units in Newtons
torque_total = zeros(length(phi),1);

for i = 1:n
    spring_length = Lsp([p(i),p(i+n)],phi);
    eps = ((spring_length - L0)/L0);
    spring_force = springForce_calculationLinear(k,eps);
    torque_total = torque_total+spring_force.* mom_arm([p(i),p(i+n)],phi);
end

end
