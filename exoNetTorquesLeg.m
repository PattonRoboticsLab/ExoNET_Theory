% ***********************************************************************
% Calculate the torques created by the ExoNET
% ***********************************************************************

function TAUs = exoNetTorquesLeg(p,PHIs)

%% Setup
global EXONET BODY

TAUs = NaN*zeros(size(PHIs)); % initialization

hipIndex = 1;
kneeIndex = EXONET.nParameters*EXONET.nElements+1;
hipKneeIndex = EXONET.nParameters*EXONET.nElements*2+1;


%% Find torques
for i = 1:size(PHIs,1)
    
    tau = 0;
    for element = 1:EXONET.nElements
        r = p(hipIndex+(element-1)*EXONET.nParameters+0);
        theta = p(hipIndex+(element-1)*EXONET.nParameters+1);
        L0 = p(hipIndex+(element-1)*EXONET.nParameters+2);
        tau = tau + tauMarionetLeg(PHIs(i,1),BODY.Lengths(1),r,theta,L0); % torque created by the Marionet
    end
    TAUs(i,1) = tau;
    
    tau = 0;
    for element = 1:EXONET.nElements
        r = p(kneeIndex+(element-1)*EXONET.nParameters+0);
        theta = p(kneeIndex+(element-1)*EXONET.nParameters+1);
        L0 = p(kneeIndex+(element-1)*EXONET.nParameters+2);
        tau = tau + tauMarionetLeg(PHIs(i,1)-PHIs(i,2),BODY.Lengths(2),r,theta,L0); % torque created by the Marionet
    end
    TAUs(i,2) = tau;
    
    if EXONET.nJoints == 3
        taus = [0 0];
        for element = 1:EXONET.nElements
            r = p(hipKneeIndex+(element-1)*EXONET.nParameters+0);
            theta = p(hipKneeIndex+(element-1)*EXONET.nParameters+1);
            L0 = p(hipKneeIndex+(element-1)*EXONET.nParameters+2);
            taus = taus + tau2jMarionetLeg(PHIs(i,:),BODY.Lengths,r,theta,L0); % torques created by the Marionet
        end
        TAUs(i,:) = TAUs(i,:) + taus;
    end

end