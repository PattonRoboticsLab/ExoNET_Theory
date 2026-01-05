% set as many parameters as possible for exonet applications
% VERSIONS:     Patton simplified & renamed setUpGravArm 2025-Sep-4
%               Patton initiated 2019-01-11
% This adds to existing; run gravComp if if you need full setup
% 
%       | PHI 2 .
%       |      .
%       |    .
%       |   .
%       |  .
%       | .
%       o
%      /  
%     /  
%    /
%   /   PHI 1
%  o . . . . . . .
%
%% ~~ BEGIN PROGRAM: ~~

fprintf('\n\n _____ ExoNET EA setup: ____ \n')

%% MARIONETS
%Exo.K=1000;        % spring Stiffness 
Exo.nParams=3;      % number of parameters governing each element
Exo.nJnts=3;        % shoulder, elbow (=2), add 2-joint (=3) 
Exo.nElements=5     % number of elements per joint 
% Exo.nElements=menu('choose # elements per joint', '1','2','3','4','5')
%tension = @(L0,L)    (Exo.K.*(L-L0)).*((L-L0)>0); % (inlineFcn) +Stretch

%% Optimization params:
p0=.05*(1:(Exo.nParams*Exo.nElements*Exo.nJnts)); % INIT.GUESS (L0,r,theta)
bestCost=1e16;                                    % init very high 
options.nTries = 50;                              % #random init restarts

%% Set the constraints for the parameters:
RLoHi = [0.00001 0.14];    % R low and high range
thetaLoHi = [-370 370];  % theta low and high range
L0LoHi = [0.05 0.30];    % L0 low and high range
i=0;
Exo.pConstraint = NaN*zeros(Exo.nJnts*Exo.nElements*Exo.nParams,2); % initialization
for joint = 1:Exo.nJnts
    for element = 1:Exo.nElements
        i = i+1;
        Exo.pConstraint(i,:) = RLoHi;
        i = i+1;
        Exo.pConstraint(i,:) = thetaLoHi;
        i = i+1;
        Exo.pConstraint(i,:) = L0LoHi;
    end
end
Exo

%% set fig& one token body pose
figure(2); clf; put_fig(2,.03,.5,.45,.42); clf; hold on; % clear & set 
set(gcf,'name','Error Augmentation')
subplot(1,2,1); drawBody3(pi/180*[-100 70], Bod); % draw at one posture

%% problem-specific for EA on the positions along traject
title('EA Field');        
[TAUsDesired,PHIs,Pos]=eaField(Bod);                % and EA field
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');        % plot desired field 
drawnow

fprintf(' parameters set. \n ')

