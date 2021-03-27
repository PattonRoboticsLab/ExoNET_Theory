% ***********************************************************************
% Calculate the torques and the tensions created by the ExoNET
% ***********************************************************************

function TAUs = exoNetTorquesTensionsLeg(p,PHIs,plotIt)

%% Setup
global EXONET BODY
if ~exist('plotIt','var'); plotIt = 0; end    % if plotIt argument not passed

TAUs = NaN*zeros(size(PHIs)); % initialization

hipIndex = 1;
kneeIndex = EXONET.nParameters*EXONET.nElements+1;
hipKneeIndex = EXONET.nParameters*EXONET.nElements*2+1;

if plotIt; cf = gcf(); figure; end            % add cf and create another figure

%% Find torques
for i = 1:size(PHIs,1)
    
    tau = 0;
    for element = 1:EXONET.nElements
        r = p(hipIndex+(element-1)*EXONET.nParameters+0);
        theta = p(hipIndex+(element-1)*EXONET.nParameters+1);
        L0 = p(hipIndex+(element-1)*EXONET.nParameters+2);
        [new_tau,EXONET.T(i,1,element),EXONET.Tdist(i,1,element)] = tauMarionetLeg(PHIs(i,1),BODY.Lengths(1),r,theta,L0);
        if plotIt
            plot(EXONET.Tdist(i,1,element),EXONET.T(i,1,element),'o','MarkerSize',7,'MarkerFaceColor',[0.5 0.7 1],'MarkerEdgeColor','w')
            xlabel('L [m]')
            ylabel('Tension [N]')
            title('Tension exerted by each elastic element')
        end
        hold on
        tau = tau + new_tau; % + element's torque
    end
    TAUs(i,1) = tau;         % torque created by the hip MARIONET
    
    tau = 0;
    for element = 1:EXONET.nElements
        r = p(kneeIndex+(element-1)*EXONET.nParameters+0);
        theta = p(kneeIndex+(element-1)*EXONET.nParameters+1);
        L0 = p(kneeIndex+(element-1)*EXONET.nParameters+2);
        [new_tau,EXONET.T(i,2,element),EXONET.Tdist(i,2,element)] = tauMarionetLeg(PHIs(i,1)-PHIs(i,2),BODY.Lengths(2),r,theta,L0);
        if plotIt
            plot(EXONET.Tdist(i,2,element),EXONET.T(i,2,element),'o','MarkerSize',7,'MarkerFaceColor',[0.1 1 0.2],'MarkerEdgeColor','w')
        end
        tau = tau + new_tau; % + element's torque
    end
    TAUs(i,2) = tau;         % torque created by the knee MARIONET
    
    if EXONET.nJoints == 3
        taus = [0 0];
        for element = 1:EXONET.nElements
            r = p(hipKneeIndex+(element-1)*EXONET.nParameters+0);
            theta = p(hipKneeIndex+(element-1)*EXONET.nParameters+1);
            L0 = p(hipKneeIndex+(element-1)*EXONET.nParameters+2);
            [new_tau,EXONET.T(i,3,element),EXONET.Tdist(i,3,element)] = tau2jMarionetLeg(PHIs(i,:),BODY.Lengths,r,theta,L0);
            if plotIt
                plot(EXONET.Tdist(i,3,element),EXONET.T(i,3,element),'o','MarkerSize',7,'MarkerFaceColor',[1 0.6 0.3],'MarkerEdgeColor','w')
                box off
            end
            taus = taus + new_tau;    % + element's torques
        end
        TAUs(i,:) = TAUs(i,:) + taus; % torques created by the hip-knee MARIONET
    end
end

if plotIt; figure(cf); end

end