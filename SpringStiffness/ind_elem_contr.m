close all
all_force = []
tension = @(L0,L) (112.2*(L/L0).^5-838.3*(L/L0).^4+2494*(L/L0).^3-3689*(L/L0).^2+2717*(L/L0)-794.2).*((L/L0)>1); 
for i = 1:size(p_2elem,1)
    element = p_2elem(i,:)
    r = element(1)
    theta = element(2)
    L0 = element(3)
for k=1:size(PHIs,1)
     Ls = Bod.L;
     phis = PHIs(k,:)
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
    true_stretch = stretch - rest;
    stretches(:,k) = true_stretch;

    Tdir=Tdir./Tdist;    % direction vector 

    if (L0 < Tdist)
    T = tension(L0,Tdist)
    else
    T = 0
    end
    spring_force(:,k) = T;
end
all_stretch(i,:) = stretches;
all_force(i,:)= spring_force;

%stretched(i) = stretch
%rested(i) = rest
end

figure()
plot(all_stretch(1,:),all_force(1,:),'bo')
hold on
plot(all_stretch(2,:),all_force(2,:),'bo')
%plot(x,a)
xlabel('Stretch (cm)')
ylabel('Tension (Nm)')
title('Elbow-Shoulder')

plot(all_stretch(3,:),all_force(3,:),'go')
hold on
plot(all_stretch(4,:),all_force(4,:),'go')
%plot(x,a)
xlabel('Stretch (cm)')
ylabel('Tension (Nm)')
title('Wrist-Elbow')

plot(all_stretch(5,:),all_force(5,:),'ro')
hold on
plot(all_stretch(6,:),all_force(6,:),'ro')
%plot(x,a)
xlabel('Stretch (cm)')
ylabel('Tension (Nm)')
title('Wrist-Shoulder')





