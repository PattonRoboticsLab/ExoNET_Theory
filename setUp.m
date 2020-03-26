 % set as many parameters as possible for the stacked marionet applications
% patton 2019-01-11
% 
%       | PHI2  .
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
%                             .X
%                       .   X  
%         TVect   .      X       lVect
%           .         X
%       o          X       
%   r  /        X
%     /      X       phi
%    /   theta        
%   /  X            
%  O .  .  .  .  .  .  .  .  .  .  .  . 

%% ~~ BEGIN PROGRAM: ~~
fprintf('\n  SetUp parameters...')
global Exo Bod PHIs TAUsDesired tension ProjectName PHIsWorkspace PosWorkspace 
close all

%% pick the problem to solve
disp('choose from menu...')
fieldType=menu('Choose a field to approximate:' ...
               , 'WeightCompensation' ...
               , 'ErrorAugmentation' ...
               , 'SingleAttractor' ...
               , 'DualAttractor' ...
               , 'LimitPush' ...
               , 'GaitTorques' ...
               , 'EXIT');     

%% MARIONETS
Exo.K=100;         % spring Stiffness 
Exo.nParams=3;      % number of parameters governing each element
Exo.nJnts=3;        % shoulder and elbow and shoulder elbow
disp('choose from menu...')
Exo.nElements=menu('number of stacked elements per joint:' ...
               , '1' ...
               , '2' ...
               , '3' ...
               , '4' ...
               , '5' ...
               , '6' );

% set desired CONSTRIANTS on the parameters: 
RLoHi=[0 .14];thetaLoHi=2*pi*[0 1];  L0LoHi=[.02 .3];        % ranges
i=0; Exo.pConstraint=NaN*zeros(Exo.nJnts*Exo.nElements*Exo.nParams,2); % init
for joint=1:Exo.nJnts
  for element=1:Exo.nElements
    i=i+1; Exo.pConstraint(i,:)=RLoHi;
    i=i+1; Exo.pConstraint(i,:)=thetaLoHi;
    i=i+1; Exo.pConstraint(i,:)=L0LoHi;
  end
end

%% Bod
Bod.M = 70;                   % body mass 
Bod.L = [.35 .26;];           % segment lengths (humerous, arm)
Bod.R = Bod.L.*[.45 .5];      % proximal to centers of mass of segments 
Bod.pose=pi/180*[-97 70];     % token body position (can be anything)

%% Setup span of full workspace posture evaluation points (angles)
nAngles = 8; % # shoulder & elbow angles in a span for evaluation
phi1=pi/180*linspace(-100,0,nAngles); phi2=pi/180*linspace(25,145,nAngles);  
PHIs=[];  
for i=1:length(phi1)          % nested 2-loop establishes grid of phi's
  for j=1:length(phi2), PHIs=[PHIs; phi1(i),phi2(j)]; end % stack up list
end 
Pos=forwardKin(PHIs,Bod);     % positions assoc w/ these angle combinations
PHIsWorkspace=PHIs;           % store this
PosWorkspace=Pos;             % store this

%% Optimization params:
optOptions=optimset();
optOptions.MaxIter = 1E3;                             % optimization limit
optOptions.MaxFunEvals = 1E3;                         % optimization limit
optimset(optOptions);
nTries = 25;%30*Exo.nElements;                            % number optim reruns 

%% HANDLE=@(ARGLIST)EXPRESSION constructs anon fcn & returns handle to it 
tension = @(L0,L)    (Exo.K.*(L-L0)).*((L-L0)>0);   % (inlineFcn) +Stretch

%% plot: 
put_fig(fieldType,.9,.35,.6,.5); subplot(1,2,1);    % place figure 
drawBody2(Bod.pose,Bod);     hold on                % draw@1 posture
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.7*[1 1 1]);% positions,grey

fprintf(' parameters set. \n ')
