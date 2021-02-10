%% Load ExoNet Data
% Initialize Parameters from Data
p = Exo.param;
K = Exo.K;
nParams = Exo.nParams;
PHIs = Exo.phis;
Pos = Exo.pos;
error = Exo.E;
nElements = Exo.nElements;

% Initialize Elastic Stiffness Function
tension = @(L0,L) (112.2*(L/L0).^5-838.3*(L/L0).^4+2494*(L/L0).^3-3689*(L/L0).^2+2717*(L/L0)-794.2).*((L/L0)>1);

%Plot Colors
Colors=  [.5 .7 1; .1 1 .2;  1 .6 .3];

%Shoulder-Elbow Index Values for Plot For Loop
shIndex=  1;
elIndex=  nParams*nElements+1;
shElIndex=nParams*nElements*2+1;
new_tau = 0;


figure()
%% Plot Stretch vs. Tension Plot
for test_point=1:size(PHIs,1) %fprintf('\n point %d..',i); % loop for each position
  
  tau=0; %fprintf(' shoulder..');  tau=tauMARIONET(phi,L,r,theta,L0)
  for element=1:Exo.nElements,  %fprintf(' element %d..',element);
    r=    p(shIndex+(element-1)*Exo.nParams+0);      % extract from p
    theta=p(shIndex+(element-1)*Exo.nParams+1);
    L0=   p(shIndex+(element-1)*Exo.nParams+2); 
    stretch_max = max(Exo.Tdist(test_point,1,element));
    x = 0:.001:stretch_max;
    y = tension(L0,x);
    plot(x,y,'Color',Colors(1,:), 'LineWidth',2.5)
    hold on
    plot(Exo.Tdist(test_point, 1, element), Exo.T(test_point, 1, element), 'o','MarkerFaceColor',Colors(1,:), 'MarkerEdgeColor', 'w','MarkerSize',8); 
    tau=tau+new_tau; % +element's torque  
  end
  TAUs(test_point,1)=tau;                                        % Storing Shoulder Torque   
  tau=0; %fprintf(' elbow..');
  for element=1:Exo.nElements,  %fprintf(' element %d..',element);
    r=     p(elIndex+(element-1)*Exo.nParams+0);      % extract from p
    theta= p(elIndex+(element-1)*Exo.nParams+1);
    L0=    p(elIndex+(element-1)*Exo.nParams+2);
    stretch_max = max(Exo.Tdist(test_point,2,element));
    x = 0:.001:stretch_max;
    y = tension(L0,x);
    plot(x,y,'Color',Colors(2,:), 'LineWidth',2.5)
    plot(Exo.Tdist(test_point, 2, element), Exo.T(test_point, 2, element), 'o','MarkerFaceColor',Colors(2,:), 'MarkerEdgeColor', 'w','MarkerSize',8);
    tau=tau+new_tau; % +element's torque
  end
  TAUs(test_point,2)=tau;                                        % Storing Elbow Torque 
  if Exo.nJnts==3, taus=[0 0]; %fprintf(' 2 joint..');     
     for element=1:Exo.nElements,  %fprintf(' element %d..',element);
      r=    p(shElIndex+(element-1)*Exo.nParams+0);   % extract from p
      theta=p(shElIndex+(element-1)*Exo.nParams+1);
      L0=   p(shElIndex+(element-1)*Exo.nParams+2);
      %[new_tau, Exo.T(test_point, 3, element), Exo.Tdist(test_point, 3, element)] = tau2jMARIONET(PHIs(test_point,:),Bod.L,r,theta,L0);
      %if plotIt, 
        stretch_max = max(Exo.Tdist(test_point,3,element));
        x = 0:.001:stretch_max;
        y = tension(L0,x);
        plot(x,y,'Color',Colors(3,:),'LineWidth',2.5)
        plot(Exo.Tdist(test_point, 3, element), Exo.T(test_point, 3, element), 'o','MarkerFaceColor',Colors(3,:), 'MarkerEdgeColor', 'w','MarkerSize',8);
        xlabel('Stretch (m)')
        ylabel('Tension (Nm)')
        title('5 Element Gravity Compensation')
      taus=taus+new_tau; % +element's torques
    end
    TAUs(test_point,:)=TAUs(test_point,:)+taus; % add 2joint calculations            
  end
end % END loop for each position

figure()
% Plot Solutions
% drawBody2(Bod.pose,Bod);                 % draw@1 posture
% plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');          % desired again
% plotVectField(PHIs,Bod,Pos,TAUs,'b');                 % plot solution 