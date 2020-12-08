
global  phi desired_torque springStiffness L_springRest  %some variables are global because of the way the code is done

%init_param=rand(1,(n*2)); %Initial guess for thetas and radii
%tau_tot=stacks(p, phi);
%cost=sum((tau_tot-desired_torque).^2);
%scatter3(p(1),p(2),cost);
%hold on



A = [desired_torque(:,1),desired_torque(:,1),desired_torque(:,1),desired_torque(:,1),desired_torque(:,1),desired_torque(:,1)];
B = [A,A,A,A,A];
C = [B,B,B,A,A,A,desired_torque(:,1),desired_torque(:,1),desired_torque(:,1),desired_torque(:,1)];
torque_desired = transpose(C);

rad_min = 0;
rad_max = .15;
ang_min = 0;
ang_max = 2*pi;

Steps = (length(phi)-1);


p(:,1) = rad_min:(rad_max-rad_min)/Steps:rad_max;
p(:,2) = ang_min:(ang_max-ang_min)/Steps:ang_max;
p(:,2) = p(:,2)*57.2958;

rad = p(:,1);
ang = p(:,2);
[X,Y] = meshgrid(rad,ang);

L = 0.26;
L0 = 0.45;
springStiffness = 100;


Y = degtorad(Y);
ang = degtorad(ang);

% for k = 1:length(phi)
%     p(1) = p(k,1);
%     p(2) = p(k,2);
%     tau_tot(:,k) = stacks(p,phi);
%     cost(:,k) = sum((tau_tot(:,k)-desired_torque).^2);
%     for i = 1:length(phi)
%         cost(i,:) = sum((tau_tot(:,i)-desired_torque).^2);
%     end
%    
% end

% for j = 1:length(p(:,1))
%     p(1) = p(j,1);
%     for i = 1:length(p(:,2))
%         p(2) = p(i,2);
%         tau_tot(:,i) = stacks(p,phi);
%     end
%     cost(j,:) = sum((tau_tot(:,j)-desired_torque).^2);
% end

for radius = 1:length(rad)
    p(2) = X(1,radius);
    for angle = 1:length(ang)
        p(1) = Y(angle,1);
        
        for k = 1:length(phi)
        tau_tot(k,:) = stacks(p,phi);
        end
        
                  
        
        cost(angle,radius) = sum((transpose(tau_tot(angle,:))-desired_torque).^2); 
        %cost(angle,radius) = 
%         if p(2)<= .01
%         cost(angle,radius)=1e4;
%         end
%     
%     if p(2) >= .0199     %Added condition on a maximum radius
%         cost(angle,radius)=1e4;
%     end
    
        
%         if p(2) <= 0.01
%             cost = 100000;
%         end
%         
%             if p(2) >= 0.05
%                 cost = 1e10;
%             end
            
        
        
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
 Radius = .1024;
 Angle = 2.899;
 Cost = 8.006;
 plot3(Radius,Angle,Cost,'o','MarkerSize',6,'MarkerFaceColor','r')

%colormap winter
%shading interp
% 
figure(2)
 plot(phi,tau_tot,'b', phi, desired_torque,'r')
 xlabel('\phi')
 ylabel('Torque')