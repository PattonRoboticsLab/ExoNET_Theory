clear all
clc

%% Load data

Data=load('Desired_torque_fake2.txt');    %loading data
phi= Data(:,1);    %angles [rad]
desired_torque = Data(:, 2); %desired torque
A = [desired_torque(:,1),desired_torque(:,1),desired_torque(:,1),desired_torque(:,1),desired_torque(:,1),desired_torque(:,1)];
B = [A,A,A,A,A];
new_torque_desired = transpose(B);

%%Radius and Angle Definition
rad_min = .05;
rad_max = .1;

ang_min = 0;
ang_max = pi;

Steps = length(phi)-1;
L = .34;
L_springRest = .45;




rad = rad_min:(rad_max-rad_min)/Steps:rad_max;
ang = ang_min:(ang_max-ang_min)/Steps:ang_max;

for k = 1:length(phi)
   
    
    for j = 1:length(rad)
    
        for i = 1:length(ang)
        
       
            tau_tot(i) = (L*rad(j).*sin(ang(i)-phi(k)))./sqrt(rad(j).^2+L^2-(2*rad(j).*L*cos(ang(i)-phi(k))));
        
        end
    
    torque(j,:) = tau_tot;
      
    end


cost(k,:) = sum((torque- new_torque_desired).^2);

end




    
figure(1)
surf(ang,rad,cost);colorbar;set(gca,'FontSize',12);
xlabel('Angle','FontName','Times','FontSize',20,'FontAngle','italic');
set(get(gca,'xlabel'),'rotation',25,'VerticalAlignment','bottom');
ylabel('Radius','FontName','Times','FontSize',20,'FontAngle','italic');
set(get(gca,'ylabel'),'rotation',-25,'VerticalAlignment','bottom');
zlabel('Cost','FontName','Times','FontSize',20,'FontAngle','italic');
title('3D View','FontName','Times','FontSize',24,'FontWeight','bold');
 
figure(2)
surf(ang,rad,cost);view(0,90);colorbar;set(gca,'FontSize',12);
xlabel('Angle','FontName','Times','FontSize',20,'FontAngle','italic');
ylabel('Radius','FontName','Times','FontSize',20,'FontAngle','italic');
zlabel('Cost','FontName','Times','FontSize',20,'FontAngle','italic');
title('Radius-Angle Plane View','FontName','Times','FontSize',24,'FontWeight','bold');
 
figure(3)
surf(ang,rad,cost);view(90,0);colorbar;set(gca,'FontSize',12);
xlabel('Angle','FontName','Times','FontSize',20,'FontAngle','italic');
ylabel('Radius','FontName','Times','FontSize',20,'FontAngle','italic');
zlabel('Cost','FontName','Times','FontSize',20,'FontAngle','italic');
title('Radius-Cost Plane View','FontName','Times','FontSize',24,'FontWeight','bold');
 
figure(4)
surf(ang,rad,cost);view(0,0);colorbar;set(gca,'FontSize',12);
xlabel('Angle','FontName','Times','FontSize',20,'FontAngle','italic');
ylabel('Radius','FontName','Times','FontSize',20,'FontAngle','italic');
zlabel('Cost','FontName','Times','FontSize',20,'FontAngle','italic');
title('Angle-Cost Plane View','FontName','Times','FontSize',24,'FontWeight','bold');
    
       