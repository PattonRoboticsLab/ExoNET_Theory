% SCRIPT to set parameters for exonet anti-gravity arm
% VERSIONS:     Patton simplified & renamed setUpGravArm 2025-Sep-4
%               Patton initiated 2019-01-11
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
%% ~~ BEGIN SCRIPT: ~~
fprintf('\n Setting Arm Gravity compentation...')

%% MARIONETS
%Exo.K=1000;         % spring Stiffness 
Exo.nParams=3;      % number of parameters governing each element
Exo.nJnts=3;        % shoulder, elbow (=2), add 2-joint (=3) 
Exo.nElements=1     % number of elements per joint 
% Exo.nElements=menu('choose # elements per joint', '1','2','3','4','5')
%tension = @(L0,L)    (Exo.K.*(L-L0)).*((L-L0)>0); % (inlineFcn) +Stretch

%% optimization
p0=.05*(1:(Exo.nParams*Exo.nElements*Exo.nJnts)); % INIT.GUESS (L0,r,theta)
bestCost=1e16;                                    % init very high 

TAUsDesired=weightEffect(Bod,Pos);                % set torques2cancelGrav

%% Set constraints on parameters:
RLoHi = [0.00001 0.14];    % R low and high range
thetaLoHi = [-370 370];    % theta low and high range
L0LoHi = [0.05 0.30];      % L0 low and high range
i=0;
Exo.pConstraint = NaN*zeros(Exo.nJnts*Exo.nElements*Exo.nParams,2); % init
for joint = 1:Exo.nJnts
    for element=1:Exo.nElements
        i=i+1; Exo.pConstraint(i,:)=RLoHi;
        i=i+1; Exo.pConstraint(i,:)=thetaLoHi;
        i=i+1; Exo.pConstraint(i,:)=L0LoHi;
    end
end
Exo

%% plot
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); % testPoints in grey
title('Gravity Compensating Field');               
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');      % plot desired field 
set(gcf,'name','Gravity'); 
jimPlot
title('force vs  deformation'); 
drawnow; pause(.01) % assures it is displayed

fprintf(' parameters set. \n ')

