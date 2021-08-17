clear
clc
warning off

%% Load Data
Data = load('Desired_torque_fake2.txt'); %loading data
phi = Data(:,1);
desired_torque = Data(:,2);
desired_torque = transpose(desired_torque);
A = [desired_torque(1,:),desired_torque(1,:),desired_torque(1,:),desired_torque(1,:),desired_torque(1,:),desired_torque(1
C = transpose(A);
B = [C,C,C,A,A];
torque_desired = transpose(B);


%% Define Limits of Radius and Angle
rad_min = 0.001;
rad_max = 0.05;
ang_min = 0;
ang_max = pi; 


Steps = length(phi)-1;

rad = rad_min:(rad_max-rad_min)/Steps:rad_max;
ang = ang_min:(ang_max-ang_min)/Steps:ang_max;

L = .26; % Length of Arm Definition
L0 = .45; %Resting Length of Spring
springStiffness = 1000;
tau_tot = zeros(length(phi),1);


for j = 1:length(rad)
    for i = 1:length(ang)
        for k = 1:length(phi)
            moment_arm(k) = (L*rad(j)*sin(ang(i)-phi(k)))./sqrt(rad(j).^2+L^2-(2*rad(j)*L*cos(ang(i)-phi(k)))); %moment arm
            spring_length(k) = (sqrt(L^2+rad(j).^2-2*L*rad(j).*cos(ang(i)-phi(k)))); %spring length as a function of the radius, angle, and phi value
            displacement(k) = ((spring_length(k)-L0)); %Displacement of Spring in Relation to Resting Length
            x = length(displacement(k)); %Length of the Spring's Displacement
            spring_force = zeros(x,1); %Initial Spring Force = 0, as it is experiences a displacement it will change

            for z = 1:x
                spring_force(z,:) = -springStiffness*displacement(k); %Spring Force Calculation
            end
            
            tau_tot(k,:) = moment_arm(k).*spring_force; %Torque Calculation: T = moment arm * spring Force
        end
        torque(i,:) = tau_tot;
        
    end

    cost(j,:)= sum((torque-torque_desired).^2);
    
end






%torque_v = transpose(torque_fin);
%% For Loop for Cost

%for k = 1:length(rad)
 %   cost(k) = sum((torque(k,:)-desired_torque).^2);
%end
 %   new_torque(:,k) = transpose(desired_torque);
    %cost(:,k) = (torque_new(:,k)-desired_torque(:,1)).^2;
    
%end

%% Plot Figures
figure(1)
surf(ang,rad,cost);colorbar;set(gca,'FontSize',12);
xlabel('Angle','FontName','Times','FontSize',20,'FontAngle','italic');
set(get(gca,'xlabel'),'rotation',25,'VerticalAlignment','bottom');
ylabel('Radius','FontName','Times','FontSize',20,'FontAngle','italic');
set(get(gca,'ylabel'),'rotation',-25,'VerticalAlignment','bottom');
zlabel('Cost','FontName','Times','FontSize',20,'FontAngle','italic');
title('3D View','FontName','Times','FontSize',24,'FontWeight','bold');
hold on
Radius = .05;
Angle = .283;
plot3(Angle,Radius,1000,'o','MarkerSize',6,'MarkerFaceColor','r')

figure(2)
subplot(2,1,1)
plot(phi,torque(1,:),phi,torque(15,:),phi,torque(30,:))
xlabel('\phi')
ylabel('Torque')
title('Torque Option for Cost')
subplot(2,1,2)
plot(phi,torque_desired(1,:))
xlabel('\phi')
ylabel('Torque')
title('Desired Torque')
            
