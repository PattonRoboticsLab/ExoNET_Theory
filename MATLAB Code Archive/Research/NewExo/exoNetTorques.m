% exoNetTorques: calulate the toreques created from a Marionet
% created 2019-Jan-23 (Patton) from Carella's stacks_mixedDevice.m
% 2019-Feb-5 fixed bug of the order of arguements going into tauMARIONET
% ~~ BEGIN PROGRAM: ~~

function TAUs=exoNetTorques(p,PHIs)

%% Setup
global Exo Bod
%TAUs=NaN*zeros(size(PHIs)); % init
% fprintf(',');

%% index parameters set from the 'p' parameters
% p vect stacked ea.joint, & within that ea.element, & ea.paramter
shIndex=  1;
elIndex=  Exo.nParams*Exo.nElements+1;
shElIndex=Exo.nParams*Exo.nElements*2+1;

%% find torques
for i=1:size(PHIs,1) %fprintf('\n point %d..',i); % loop for each position
  
  tau=0; %fprintf(' shoulder..');  tau=tauMARIONET(phi,L,r,theta,L0)
  for element=1:Exo.nElements,  %fprintf(' element %d..',element);
    r=    p(shIndex+(element-1)*Exo.nParams+0);     % index into p
    theta=p(shIndex+(element-1)*Exo.nParams+1);
    L0=   p(shIndex+(element-1)*Exo.nParams+2);
    tau=tau+tauMARIONET(PHIs(i,1),Bod.L(1),r,theta,L0); % +element's torque  
  end
  TAUs(i,1)=tau;
  
  tau=0; %fprintf(' elbow..');
  for element=1:Exo.nElements,  %fprintf(' element %d..',element);
    r=     p(elIndex+(element-1)*Exo.nParams+0);      % index into p
    theta= p(elIndex+(element-1)*Exo.nParams+1);
    L0=    p(elIndex+(element-1)*Exo.nParams+2);
    tau=tau+tauMARIONET(PHIs(i,2),Bod.L(2),r,theta,L0); % +element's torque
  end
  TAUs(i,2)=tau;
  
  if Exo.nJnts==3, taus=[0 0]; %fprintf(' 2 joint..');     
     for element=1:Exo.nElements,  %fprintf(' element %d..',element);
      r=    p(shElIndex+(element-1)*Exo.nParams+0);   % index into p
      theta=p(shElIndex+(element-1)*Exo.nParams+1);
      L0=   p(shElIndex+(element-1)*Exo.nParams+2);
      taus=taus+tau2jMARIONET(PHIs(i,:),Bod.L,r,theta,L0); % +element's torques
    end
    TAUs(i,:)=TAUs(i,:)+taus; % add this in 
  end
  
end % END loop for each position

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
