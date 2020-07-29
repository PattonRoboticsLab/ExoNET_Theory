% caluate the desired torques needed from weight cancellation
% patton 2019-01-14

function [tauD,PHIs,Pos]=weightEffect(Bod,Pos,showIt)
global ProjectName 
if ~exist('showIt','var'); showIt=0; end % default
ProjectName='GravityCompensatingField';
title(ProjectName);
fprintf('\n - %s : - \n',ProjectName)

%% Setup span of full workspace posture evaluation points (angles)
nAngles = 5; % # shoulder & elbow angles in a span for evaluation
phi1=pi/180*linspace(-100,0,nAngles); phi2=pi/180*linspace(25,145,nAngles);  
PHIs=[];  
for i=1:length(phi1)          % nested 2-loop establishes grid of phi's
  for j=1:length(phi2), PHIs=[PHIs; phi1(i),phi2(j)]; end % stack up list
end 
Pos=forwardKin(PHIs,Bod);     % positions assoc w/ these angle combinations

%% weights
g= 9.81;                                                % gravity constant
hand_weight       = .181437*g;%(0.61/100)*Bod.M*g;                 % from winter's book
foreArm_weight    = .453592*g;%(1.62/100)*Bod.M*g;
forearmHandWeight = foreArm_weight+hand_weight;         % sum 'cause they're 1`
upperArm_weight   = .635029*g;%(2.71/100)*Bod.M*g;
Bod.weights       = [upperArm_weight, foreArm_weight, hand_weight];
for i=1:size(PHIs,1) % torques from weights:              % loop ea config
  tauSh=cross([Pos.R1(i,:) 0],[0 -upperArm_weight 0])+ ...% 3Dvects 4this
        cross([Pos.R2(i,:) 0],[0 -forearmHandWeight 0]);    %  demand/desired
  tauEl=cross([Pos.el2R2(i,:) 0],[0 -forearmHandWeight 0]);% simmilar4elbow
  tauD(i,:)=-[tauSh(:,3) tauEl(:,3)];                      % 3rd dim 4torque
%   equivWristForce=(inv(jacobian(PHIs(i,:),Bod.L)')*tauD(i,:)')'; % Force (not needed here)
end

if showIt
  plotVectField(PHIs,Bod,Pos,tauD,'r')
end
