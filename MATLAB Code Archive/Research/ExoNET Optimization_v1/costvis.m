
global  phi desired_torque springStiffness L_springRest  %some variables are global because of the way the code is done

%% Set Range for Radius and Theta ANgle
rad_min = 0.010001;
rad_max = .0150001;
ang_min = 0;
ang_max = 2*pi;

Steps = (length(phi)-1);

p(:,1) = rad_min:(rad_max-rad_min)/Steps:rad_max;
p(:,2) = ang_min:(ang_max-ang_min)/Steps:ang_max;
p(:,2) = p(:,2)*57.2958;

%% Create a Meshgrid so that all possible values and combinations of the radius and theta angle are accounted for
rad = p(:,1);
ang = p(:,2);
[X,Y] = meshgrid(rad,ang);

%%Specify Length and Stiffness parameters
L = 0.26;
L0 = 0.45;
springStiffness = 100;

%%Optional, re-convert everything to radians so that you're keeping
%%consistent
Y = degtorad(Y);
ang = degtorad(ang);

%% Nested For Loop for determining Cost Function Value
for radius = 1:length(rad)
    p(2) = X(1,radius);
    for angle = 1:length(ang)
        p(1) = Y(angle,1);
        
        for k = 1:length(phi)
        tau_tot(k,:) = stacks(p,phi);
        end              
        
        cost(angle,radius) = sum((transpose(tau_tot(angle,:))-desired_torque).^2); 
        
        if p(2)<= .01
        cost(angle,radius)=2.5e4;
        end
    
        if p(2) >= .05     %Added condition on a maximum radius
        cost(angle,radius)= 2.5e4;
        end    
              
    end
end

       

    
       
         
    

      



%for i = 1:length(tau_tot)
 %  cost(:,i) = sum((tau_tot(:,i)-desired_torque).^2);
%end
figure(1)
surf(rad,ang,cost);colorbar;set(gca,'FontSize',12);
grid off
xlabel('Radius (m)','FontName','Times','FontSize',20,'FontAngle','italic');
set(get(gca,'xlabel'),'rotation',25,'VerticalAlignment','bottom');
ylabel('\phi (Rad)','FontName','Times','FontSize',20,'FontAngle','italic');
set(get(gca,'ylabel'),'rotation',-25,'VerticalAlignment','bottom');%
zlabel('Cost','FontName','Times','FontSize',20,'FontAngle','italic');
title('3D View','FontName','Times','FontSize',24,'FontWeight','bold');
hold on
 Radius = .01193;
 Angle = 3.51;
 Cost = 2,422;
 plot3(Radius,Angle,Cost,'o','MarkerSize',6,'MarkerFaceColor','r')

%colormap winter
%shading interp
% 
figure(2)
 plot(phi,tau_tot,'b', phi, desired_torque,'r')
 xlabel('\phi')
 ylabel('Torque')