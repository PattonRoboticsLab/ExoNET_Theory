% ***********************************************************************
% Compute the Work and the Strain Energy
% ***********************************************************************


%% WORK
% Work [Joule] = Torque [Nm] x angle [rad]

Residual = TAUsDESIRED - TAUs; % to calculate the Residual Torque

PHIS = deg2rad(PHIs);
deltaPHI(1,:) = [0 0];
for i = 2 : length(PHIS)
    deltaPHI(i,:) = PHIS(i,:) - PHIS(i-1,:);
end

Wmuscles = sum(sum(abs(TAUsDESIRED).*abs(deltaPHI),1));

Wresidual = sum(sum(abs(Residual).*abs(deltaPHI),1));

WexoNET = sum(sum(abs(TAUs).*abs(deltaPHI),1));

percentageRemainMuscleWork = (Wresidual/Wmuscles)*100;

percentageExoNETWork = (WexoNET/Wmuscles)*100;


%% STRAIN ENERGY
% Strain Energy [Joule] = 1/2 x Force [N] x Stretch [m]

L0 = p(3:3:end); % resting lengths

stretch = NaN*ones(size(PHIs,1),3,3);
stretch(:,1,1) = EXONET.Tdist(:,1,1) - L0(1);
stretch(:,1,2) = EXONET.Tdist(:,1,2) - L0(2);
stretch(:,1,3) = EXONET.Tdist(:,1,3) - L0(3);
stretch(:,2,1) = EXONET.Tdist(:,2,1) - L0(4);
stretch(:,2,2) = EXONET.Tdist(:,2,2) - L0(5);
stretch(:,2,3) = EXONET.Tdist(:,2,3) - L0(6);
stretch(:,3,1) = EXONET.Tdist(:,3,1) - L0(7);
stretch(:,3,2) = EXONET.Tdist(:,3,2) - L0(8);
stretch(:,3,3) = EXONET.Tdist(:,3,3) - L0(9);

strainEnergy = sum(sum(sum(EXONET.T.*stretch)));

strainEnergy/Wmuscles

