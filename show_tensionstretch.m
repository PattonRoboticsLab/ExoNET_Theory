function show_tensionstretch(PHIs,Bod,p)
    global tension
    all_force = []
    %Set the optimized parameters of radius, theta, and L0 
    for i = 1:3:length(p);
        r = p(i)
        theta = p(i+1)
        L0 = p(i+2)
    for k=1:size(PHIs,1)
         Ls = Bod.L;
         phis = PHIs(k,:);
         rVect=[r*cos(theta)       r*sin(theta)       0];  % vect to rotatorPin
        %norm_r = norm(rVect);
        elbow=[Ls(1)*cos(phis(1)) Ls(1)*sin(phis(1)) 0];  % elbow pos
        wrist=[elbow(1)+Ls(2)*cos(phis(1)+phis(2)), ...   % wrist pos
           elbow(2)+Ls(2)*sin(phis(1)+phis(2)), ...
           0];
        elbow2wr=wrist-elbow;                             % elbow to wrist vect
        Tdir=rVect-wrist;                                 % tension element vector 
        Tdist=norm(Tdir);                                 % length, rotator2endpt
        stretch = Tdist;
        rest = L0;
        true_stretch = stretch;
        stretches(:,k) = true_stretch/(L0);

        Tdir=Tdir./Tdist;    % direction vector 

        if (L0 > 0)
        T = tension(L0,Tdist);
        else
        T = 0;
        end
        spring_force(:,k) = T;
     end
    all_stretch(i,:) = stretches;
    all_force(i,:)= spring_force;
    end
figure()
for i = 1:size(all_stretch,1)
plot(all_stretch(i,:),all_force(i,:),'o','MarkerSize',7,'MarkerFaceColor','k','MarkerEdgeColor','w')
hold on
%xlim([0 10])
%ylim([-0.4 40])
xlabel('Stretch Ratio')
ylabel('Tension (Nm)')
title('Stretch vs Tension on Bungee Cords with Optimized Parameter Values')
%text(x(1),y(1),'   max')
end
% plot(all_stretch(3,:),all_force(3,:),'go')
% hold on
% plot(all_stretch(4,:),all_force(4,:),'go')
% %plot(x,a)
% xlabel('Stretch (cm)')
% ylabel('Tension (Nm)')
% title('Wrist-Elbow')
% 
% plot(all_stretch(5,:),all_force(5,:),'ro')
% hold on
% plot(all_stretch(6,:),all_force(6,:),'ro')
% %plot(x,a)
% xlabel('Stretch (cm)')
% ylabel('Tension (Nm)')
% title('Wrist-Shoulder')





