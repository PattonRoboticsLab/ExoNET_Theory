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
%% ~~ BEGIN PROGRAM: ~~
fprintf('\n  SetUp parameters...')
global Exo Bod PHIs TAUsDesired tension ProjectName PHIsWorkspace PosWorkspace
close all

%% MARIONETS
Exo.K=500;          % spring Stiffness 
Exo.nParams=3;      % number of parameters governing each element
Exo.nJnts=3;        % shoulder and elbow and shoulder elbow

disp('choose from menu...')
Exo.nElements=menu('number of stacked elements per joint:' ...
               , '1' ...
               , '2' ...
               , '3' ...
               , '4' ...
               , '5')
  
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
options=optimset();
options.MaxIter = 1E3;                              % optimization limit
options.MaxFunEvals = 1E3;                          % optimization limit
nTries = 10;                                       % number optim reruns 
p0=.05*(1:(Exo.nParams*Exo.nElements*Exo.nJnts));   % INIT.GUESS (L0,r,theta)
bestCost=1e7;                                       % init very high 

%% HANDLE=@(ARGLIST)EXPRESSION constructs anon fcn & returns handle to it 
tension = @(L0,L)    (Exo.K.*(L-L0)).*((L-L0)>0);   % (inlineFcn) +Stretch

%% plot: 
put_fig(fieldType,.99,.35,.6,.5); subplot(1,2,1);    % place figure 
drawBody2(Bod.pose,Bod);     hold on                % draw@1 posture
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.7*[1 1 1]);% positions,grey

fprintf(' parameters set. \n ')

