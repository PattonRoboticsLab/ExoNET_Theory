data = load('Desired_torque_fake2.txt'); %loading data
phi = data(:,1);
desired_torque = data(:,2);
L = 0.25;

for angle = 0:3
    for radius = 0:3
    tau_tot = L*radius*sin(angle-phi)./sqrt(radius.^2+L^2-(2*radius*L*cos(angle-phi)));
    cost=sum((tau_tot-desired_torque).^2);  
    end
end
