%calculate the desired torques needed from weight cancellation

function tauD = weightEffect_n(Bod, Pos, showIt)

global PHIs


if ~exist('showIt','var'); 
    showIt = 0; 
end %default

%% Weights
g = 9.81; %gravity constant
hand_weight = (0.61/100)*Bod.M*g; %calculating hand weight based on winter's book
foreArm_weight =  (1.62/100)*Bod.M*g; %calculating forearm weight based on winter's book
forearmHandWeight = foreArm_weight + hand_weight; %summing them together because they are one
upperArm_weight = (2.71/100)*Bod.M*g;
Bod.weights = [upperArm_weight, foreArm_weight, hand_weight];
scaleF = 0.002; %graphical scale factor for gforce vectors

% plot([R1Pos(iShow,1) R2Pos(iShow,1)], [R1Pos(iShow,2) R2Pos(iShow,2)], 'k.') %plot CM's
% simpleArrow(Pos.R1(iShow,:), Pos.R1(iShow,:) - scaleF*[0 upperArm_weight],'m',1);
% simpleArrow(Pos.R2(iShow,:), Pos.R2(iShow,:) - scaleF*[0 foreArm_weight],'m',1);

%torques below are being determined by taking the cross product between the
%Pos.R1 matrix and the upper arm weight
for i = 1:size(PHIs,1) %torques from weights:  % loop ea config
    tauSh = cross([Pos.R1(i,:) 0],[0 -upperArm_weight 0]) + ... % 3Dvect 4 this
            cross([Pos.R2(i,:) 0],[0 -forearmHandWeight 0]); %demand/desired
    tauEl = cross([Pos.el2R2(i,:) 0],[0 -forearmHandWeight 0]); %similar4elbow
    tauD(i,:) = - [tauSh(:,3) tauEl(:,3)]; %3rd dim 4torque
    equivWristForce = (inv(jacobian(PHIs(i,:),Bod.L)') * tauD(i,:)')'; % Force
    if showIt,
        simpleArrow(Pos.wr(i,:),Pos.wr(i,:)+scaleF*equivWristForce,'y',1);
    end
end

h=3
