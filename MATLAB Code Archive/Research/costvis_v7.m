clear
clc
warning off

% Load Data
Data=load('Desired_torque_fake2.txt');    %loading data
phi= Data(:,1);    %angles [rad]
desired_torque = Data(:, 2);

% Define Limits of Radius and Angle
rad_min=.05;
rad_max=.1;
ang_min=0;
ang_max=pi;

rad=rad_min:(rad_max-rad_min)/(length(phi)-1):rad_max;
ang=ang_min:(ang_max-ang_min)/(length(phi)-1):ang_max;

L = .26; % Length of Arm Definition
L0 = .45;
springStiffness = 1000;
     
for j=1:length(rad)
    
    for i=1:length(ang)
      
        moment_arm(i) = (L*rad(j).*sin(ang(i)-phi(i)))./sqrt(rad(j).^2+L^2-(2*rad(j).*L*cos(ang(i)-phi(i))));   
        spring_length(i) = (sqrt(L^2+rad(j).^2-2*L*rad(j).*cos(ang(i)-phi(i))));
        displacement(i) = ((spring_length(i) - L0)./L0);
        spring_force(i) = -springStiffness*displacement(i);
            
         
        torque(i,:)= moment_arm(i).*spring_force(i);%(spring_length(k)-Lsp_rest).*(springStiffness);
        
        end
        
        tau_tot(j,:) = torque;
    end
    
    
 



% For Loop for Cost

for k = 1:length(phi)
    new_torque(:,k) = desired_torque;
    cost(k,:) = (new_torque(:,k)-tau_tot(:,k)).^2;
    
end

figure(1)
surf(ang,rad,cost);colorbar;set(gca,'FontSize',12);
xlabel('Angle','FontName','Times','FontSize',20,'FontAngle','italic');
set(get(gca,'xlabel'),'rotation',25,'VerticalAlignment','bottom');
ylabel('Radius','FontName','Times','FontSize',20,'FontAngle','italic');
set(get(gca,'ylabel'),'rotation',-25,'VerticalAlignment','bottom');
zlabel('Cost','FontName','Times','FontSize',20,'FontAngle','italic');
title('3D View','FontName','Times','FontSize',24,'FontWeight','bold');