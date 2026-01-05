% set as many parameters as possible for exonet applications
% VERSIONS:     Patton initiated from setUpGravArm 2025-OCt-4
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

fprintf('\n\n _____ ExoNET drawn field setup: ____ \n')

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

%% set full span of posture evaluation points (angles)
nAngles = 7; % # shoulder & elbow angles in a span for evaluation
phi1=pi/180*linspace(-100,0,nAngles); phi2=pi/180*linspace(30,145,nAngles);  
PHIs=[];  
for i=1:length(phi1), % nested 2 loop establishes grid of phi combinations
  for j=1:length(phi2), PHIs=[PHIs; phi1(i),phi2(j)]; end; % stack up list
end; 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); % testPoints in grey

bestCost=1e16;                                    % init very high 

%% problem-specific for EA on the positions along traject
[TAUsDesired,PHIs,Pos]=drawnField(Bod);
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');        % plot desired field 
drawnow

%% plot
set(gcf,'name','Drawn')
title('You draw the Desired Field'); 
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); % testPoints in grey
title('Gravity Compensating Field');               
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');      % plot desired field 
set(gcf,'name','Gravity'); 
jimPlot
title('force vs  deformation'); 
drawnow; pause(.01) % assures it is displayed

fprintf(' parameters set. \n ')

