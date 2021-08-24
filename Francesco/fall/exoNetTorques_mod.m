% exoNetTorques: calulate the toreques created from a Marionet
% created 2019-Jan-23 (Patton) from Carella's stacks_mixedDevice.m
% 2019-Feb-5 fixed bug of the order of arguements going into tauMARIONET
% 2019-Feb-10 added pot of marionets
% ~~ BEGIN PROGRAM: ~~

function TAUs=exoNetTorques_mod(p,PHIs, plotIt)

%% Setup
global Exo Bod tension
if ~exist('plotIt', 'var'), plotIt = 0; end     % if plotIt argument not passed
Colors=  [.5 .7 1; .1 1 .2;  1 .6 .3];
TAUs=NaN*zeros(size(PHIs)); % init
% indexes-p vect stacked ea.joint, & within that ea.element, & ea.paramter:
shIndex=  1;
elIndex=  Exo.nParams*Exo.nElements+1;
shElIndex=Exo.nParams*Exo.nElements*2+1;
if plotIt, cf = gcf(); figure(40); end    % Add cf and switch to another figure

%% find torques
for test_point=1:size(PHIs,1) %fprintf('\n point %d..',i); % loop for each position
  
  tau=0; %fprintf(' shoulder..');  tau=tauMARIONET(phi,L,r,theta,L0)

  for element=1:Exo.nElements,  %fprintf(' element %d..',element);
    r0=     p(shIndex+(element-1)*Exo.nParams+0);      % extract from p %all the parameters
    theta0= p(shIndex+(element-1)*Exo.nParams+1);
    r1=     p(shIndex+(element-1)*Exo.nParams+2);      
    theta1= p(shIndex+(element-1)*Exo.nParams+3);
    L0=     p(shIndex+(element-1)*Exo.nParams+4);
    %[new_tau, T, Tdist] = tauMARIONET(PHIs(i,1),Bod.L(1),r,theta,L0); %
    [new_tau, Exo.T(test_point, 1, element), Exo.Tdist(test_point, 1, element)] = tauMARIONET_mod(PHIs(test_point,1),Bod.L(1),r0,theta0,r1,theta1,L0); %
    if plotIt, 
    
    
    stretch_max = max(Exo.Tdist(test_point,1,element));
    x = 0:.001:stretch_max;
    y = tension(L0,x);
    plot(x,y,'Color',Colors(1,:), 'LineWidth',2.5)
    hold on
    plot(Exo.Tdist(test_point, 1, element), Exo.T(test_point, 1, element), 'o','MarkerFaceColor',Colors(1,:), 'MarkerEdgeColor', 'w','MarkerSize',8); 
    end
    tau=tau+new_tau; % +element's torque  
  end
  TAUs(test_point,1)=tau;                                        % Storing Shoulder Torque 
 
  tau=0; %fprintf(' elbow..');
  for element=1:Exo.nElements,  %fprintf(' element %d..',element);
    r0=     p(elIndex+(element-1)*Exo.nParams+0);      % extract from p %all the parameters
    theta0= p(elIndex+(element-1)*Exo.nParams+1);
    r1=     p(elIndex+(element-1)*Exo.nParams+2);      
    theta1= p(elIndex+(element-1)*Exo.nParams+3);
    L0=     p(elIndex+(element-1)*Exo.nParams+4);
    [new_tau, Exo.T(test_point, 2, element), Exo.Tdist(test_point, 2, element)] = tauMARIONET_mod(PHIs(test_point,2),Bod.L(2),r0,theta0,r1,theta1,L0); %
    if plotIt, 
    
    stretch_max = max(Exo.Tdist(test_point,2,element));
    x = 0:.001:stretch_max;
    y = tension(L0,x);
    plot(x,y,'Color',Colors(2,:), 'LineWidth',2.5)
    plot(Exo.Tdist(test_point, 2, element), Exo.T(test_point, 2, element), 'o','MarkerFaceColor',Colors(2,:), 'MarkerEdgeColor', 'w','MarkerSize',8);
    
    end
    tau=tau+new_tau; % +element's torque
  end
   TAUs(test_point,2)= tau;                                        % Storing Elbow Torque 
  
  if Exo.nJnts==3, taus=[0 0]; %fprintf(' 2 joint..');     
     for element=1:Exo.nElements,  %fprintf(' element %d..',element);
      r0=    p(shElIndex+(element-1)*Exo.nParams+0);   % extract from p
      theta0=p(shElIndex+(element-1)*Exo.nParams+1);
      r2=    p(shElIndex+(element-1)*Exo.nParams+2);   % extract from p
      theta2=p(shElIndex+(element-1)*Exo.nParams+3);
      L0=   p(shElIndex+(element-1)*Exo.nParams+4);
      [new_tau, Exo.T(test_point, 3, element), Exo.Tdist(test_point, 3, element)] = tau2jMARIONET_mod(PHIs(test_point,:),Bod.L,r0,theta0,r2,theta2,L0);
      if plotIt, 
        stretch_max = max(Exo.Tdist(test_point,3,element));
        x = 0:.001:stretch_max;
        y = tension(L0,x);
        plot(x,y,'Color',Colors(3,:),'LineWidth',2.5)
        plot(Exo.Tdist(test_point, 3, element), Exo.T(test_point, 3, element), 'o','MarkerFaceColor',Colors(3,:), 'MarkerEdgeColor', 'w','MarkerSize',8);
        xlabel('Stretch (m)')
        ylabel('Tension (Nm)')
        title('Stretch vs  Tension on ExoNET Elements')
      
      
      end

      
      taus=taus+new_tau; % +element's torques
    end
    TAUs(test_point,:)=TAUs(test_point,:)+taus; % add 2joint calculations            
  end
  

end % END loop for each position

if plotIt, figure(cf); savefig('tensionstretch.fig');  end
%% this was intitial development for a 2-joint with single elements on each
% for i=1:size(PHIs,1) % loop for each position
%   
%   fprintf(' shoulder..');
%   r=p(1); theta=p(2); L0=p(3);
%   TAUs(i,1)=tauMARIONET(PHIs(i,1),Bod.L(1),r,theta,L0);
% 
%   fprintf(' elbow..');
%   r=p(4); theta=p(5); L0=p(6);
%   TAUs(i,2)=tauMARIONET(PHIs(i,2),Bod.L(2),r,theta,L0);
%   
% end
% return