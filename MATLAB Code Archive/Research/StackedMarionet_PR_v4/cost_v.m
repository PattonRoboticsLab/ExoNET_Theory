global phi desired_torque n

tau_tot=stacks(p, phi); 
cost=sum((tau_tot-desired_torque).^2);