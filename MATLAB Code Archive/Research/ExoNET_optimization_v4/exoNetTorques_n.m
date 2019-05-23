%% ExoNETtorques: calculate the torques created from an ExoNET

function TAUs = exoNetTorques_n(p,PHIs)

%%Setup
global Exo Bod
%TAUs = NaN*zeros(size(PHIs)); %init
%fprintf(',');

%% index parameters set from the 'p' parameters
%p vector stacked ea.joint, & within that ea.element, & ea.parameter

shIndex = 1;
elIndex = Exo.nParams*Exo.nElements+1;
shElIndex = Exo.nParams*Exo.nElements*2+1;

%% Find Torques
for i = 1:size(PHIs,1) %fprintf('\n point %d..',i); %loop for each position
    tau = 0; %fprintf('shoulder..'); tau=tauMARIONET(phi,L,r,theta,L0)
    
    for element = 1:Exo.nElements, %fprintf(' element %d..', element);
        %p(shIndex+(element-1)*Exo.nParams+0)= 0.05;
        r = .05; %p(shIndex+(element-1)*Exo.nParams+0);
        theta = p(element*2-1);
        L0 = p(element*2);
%         theta = p(shIndex+(element-1)*Exo.nParams+0);
%         L0 = p(shIndex+(element-1)*Exo.nParams+1);
        tau = tau+tauMARIONET(PHIs(i,1),Bod.L(1),r,theta,L0);
    end
    
    TAUs(i,1) = tau;
    
    tau = 0;
    for element = 1:Exo.nElements, %fprintf(' element %d..', element);
        %p(elIndex+(element-1)*Exo.nParams+0) = 0.05;
        r = .05; %p(elIndex+(element-1)*Exo.nParams+0);
        theta = p(element*2-1);
        L0 = p(element*2);
%         theta = p(elIndex+(element-1)*Exo.nParams+0);
%         L0 = p(elIndex+(element-1)*Exo.nParams + 1);
        tau = tau + tauMARIONET(PHIs(i,2), Bod.L(2), r, theta, L0); %+element's torque
    end
    
    TAUs(i,2) = tau;
    
    if Exo.nJnts == 3, taus = [0,0]; %fprintf(' 2 joint..');
        for element = 1:Exo.nElements, %fprintf(' element %d..', element);
            %p(shElIndex+(element-1)*Exo.nParams+0) = 0.05;
            r = 0.05;%p(shElIndex+(element-1)*Exo.nParams+0);
            theta = (element*2-1);
            L0 = (element*2);
%             theta = p(shElIndex+(element-1)*Exo.nParams+0);
%             L0 = p(shElIndex + (element-1)*Exo.nParams+1);
            taus = taus + tau2jMARIONET(PHIs(i,:),Bod.L, r, theta, L0); %+ element's torques
        end
        TAUs(i,:) = TAUs(i,:) + taus; %add this in
    end
end %end loop for each position

%% this was initial development for a 2-joint with single elements on each
% for i=1:size(PHIs,1) % loop for each position
%   
%   fprintf(' shoulder..');
%   r=0.05; theta=p(1); L0=p(2);
%   TAUs(i,1)=tauMARIONET(PHIs(i,1),Bod.L(1),r,theta,L0);
% 
%   fprintf(' elbow..');
%   r=0.05; theta=p(3); L0=p(4);
%   TAUs(i,2)=tauMARIONET(PHIs(i,2),Bod.L(2),r,theta,L0);
%   
% end
% return
