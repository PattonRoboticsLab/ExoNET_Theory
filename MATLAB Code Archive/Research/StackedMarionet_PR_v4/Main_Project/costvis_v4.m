clear
clc
warning off

%% Load Data
Data = load('Desired_torque_fake2.txt'); %loading data
phi = Data(:,1);
desired_torque = Data(:,2);

%% Define Limits of Radius and Angle
rad_min = 0.05;
rad_max = .5;
ang_min = 0;
ang_max = pi;

rad = rad_min:(rad_max-rad_min)/(length(phi)-1):rad_max;
ang = ang_min:(ang_max-ang_min)/(length(phi)-1):ang_max;

L = .26; % Length of Arm Definition
L0 = .45;
springStiffness = 1000;



for j = 1:length(rad)
    for i = 1:length(ang)
        for k = 1:length(phi)
            moment_arm(k) = (L*rad(j)*sin(ang(i)-phi(k)))./sqrt(rad(j).^2+L^2-(2*rad(j)*L*cos(ang(i)-phi(k))));
            spring_length(k) = (sqrt(L^2+rad(j).^2-2*L*rad(j).*cos(ang(i)-phi(k))));
            displacement(k) = ((spring_length(k)-L0)./L0);
            x = length(displacement(k));
            spring_force = zeros(x,1);

            for z = 1:x
                spring_force(z,:) = -springStiffness*displacement(k);
            end
            
            tau_tot(k,:) = moment_arm(k).*spring_force;
        end
        torque(i,:) = tau_tot;
    end
    
end

torque_v = transpose(torque_fin);
%% For Loop for Cost

for k = 1:length(phi)
    new_torque(:,k) = transpose(desired_torque);
    cost(:,k) = (new_torque(:,k)-torque_v(:,k)).^2;
    
end

%% Plot Figures
figure(1)
surf(ang,rad,cost);colorbar;set(gca,'FontSize',12);
xlabel('Angle','FontName','Times','FontSize',20,'FontAngle','italic');
set(get(gca,'xlabel'),'rotation',25,'VerticalAlignment','bottom');
ylabel('Radius','FontName','Times','FontSize',20,'FontAngle','italic');
set(get(gca,'ylabel'),'rotation',-25,'VerticalAlignment','bottom');
zlabel('Cost','FontName','Times','FontSize',20,'FontAngle','italic');
title('3D View','FontName','Times','FontSize',24,'FontWeight','bold');

            
