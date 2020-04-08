% ***********************************************************************
% Calculate the torques created and the energy stored by the ExoNET
% ***********************************************************************

function [TAUs,PEs] = exoNetTorquesLeg(p,PHIs)

%% Setup
global EXONET BODY

TAUs = NaN*zeros(size(PHIs));    % initialization
PEs = NaN*zeros(length(PHIs),5); % initialization

hipIndex = 1;
kneeIndex = EXONET.nParameters*EXONET.nElements+1;
hipKneeIndex = EXONET.nParameters*EXONET.nElements*2+1;


%% Find torques and Energy
for i = 1:size(PHIs,1)
    
    tau = 0;
    pe = 0;
    for element = 1:EXONET.nElements
        r = p(hipIndex+(element-1)*EXONET.nParameters+0);
        theta = p(hipIndex+(element-1)*EXONET.nParameters+1);
        L0 = p(hipIndex+(element-1)*EXONET.nParameters+2);
        [temp_tau,temp_pe] = tauMarionetLeg(PHIs(i,1),BODY.Lengths(1),r,theta,L0); % calculate torque and energy
        tau = tau + temp_tau;
        pe = pe + temp_pe;
    end
    TAUs(i,1) = tau; % torque created by the hip MARIONET
    PEs(i,1) = pe;   % energy stored by the hip MARIONET
    
    tau = 0;
    pe = 0;
    for element = 1:EXONET.nElements
        r = p(kneeIndex+(element-1)*EXONET.nParameters+0);
        theta = p(kneeIndex+(element-1)*EXONET.nParameters+1);
        L0 = p(kneeIndex+(element-1)*EXONET.nParameters+2);
        [temp_tau,temp_pe] = tauMarionetLeg(PHIs(i,1)-PHIs(i,2),BODY.Lengths(2),r,theta,L0); % calculate torque and energy
        tau = tau + temp_tau;
        pe = pe + temp_pe;
    end
    TAUs(i,2) = tau; % torque created by the knee MARIONET
    PEs(i,2) = pe;   % energy stored by the knee MARIONET

    
    if EXONET.nJoints == 3
        taus = [0 0];
        pe = 0;
        for element = 1:EXONET.nElements
            r = p(hipKneeIndex+(element-1)*EXONET.nParameters+0);
            theta = p(hipKneeIndex+(element-1)*EXONET.nParameters+1);
            L0 = p(hipKneeIndex+(element-1)*EXONET.nParameters+2);
            [temp_taus,temp_pe] = tau2jMarionetLeg(PHIs(i,:),BODY.Lengths,r,theta,L0); % calculate torque and energy
            taus = taus + temp_taus;
            pe = pe + temp_pe;
        end
        TAUs(i,:) = TAUs(i,:) + taus; % torques created by the hip-knee MARIONET
        PEs(i,3) = pe;                % energy stored by the hip-knee MARIONET
    end
end

% PEs(:,4) = sum(PEs,2); % (does NOT work) total energy stored by the ExoNET
for i = 1:length(PEs)
    PEs(i,4) = PEs(i,1)+PEs(i,2)+PEs(i,3); % total energy stored by the ExoNET
end
PEs(:,5) = PEs(:,4)./PEs(27,4); % to express the Potential Energy as a fraction of
                                % the Potential Energy at the moment before heel contact

end