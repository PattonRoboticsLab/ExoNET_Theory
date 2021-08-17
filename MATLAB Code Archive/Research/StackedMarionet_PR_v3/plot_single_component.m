function plot_single_component(param)

global phi n L Lsp k_sp Lsp_rest

k = k_sp; %spring stiffness N/m

for i = 1:n
    spring_length = Lsp([param(i),param(i+n)],phi);
    L0 = Lsp_rest;
    displacement = ((spring_length - L0));
    F_spring = springForce_calculationLinear(k,displacement);
    mom_arm = @(p,phi) F_spring * (L*p(2)*sin(p1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)));
    hold on
    t_ind = mom_arm(param(i),param(i+n)],phi);
    plot(phi,t_ind,'c');
    text('Marionet',num2str(i))
end
end
