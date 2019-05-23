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
global Exo Bod PHIs TAUsDesired tension 
put_fig(5,.035,.113,.75,.8); subplot(1,2,1) % clear and set display:
put_fig(4,.035,.113,.75,.8); subplot(1,2,1) % clear and set display:
put_fig(3,.035,.113,.75,.8); subplot(1,2,1) % clear and set display:
put_fig(2,.035,.113,.75,.8); subplot(1,2,1) % clear and set display:
put_fig(1,.025,.11,.75,.8); subplot(1,2,1) % clear and set display:


%% MARIONETS
fprintf('\n  Set marionets...')
Exo.K=500;          % spring Stiffness 
Exo.nParams=3;      % number of parameters governing each element
Exo.nJnts=3;        % shoulder and elbow and shoulder elbow 
Exo.nElements=3;     % number of stacked elements per joint

%% Bod
Bod.M = 70;                   % body mass 
Bod.L = [.35 .26;];           % segment lengths (humerous, arm)
Bod.R = Bod.L.*[.45 .5];      % proximal to centers of mass of segments 

%% set full span of posture evaluation points (angles)
nAngles = 5; % # shoulder & elbow angles in a span for evaluation
phi1=pi/180*linspace(-100,0,nAngles); phi2=pi/180*linspace(25,145,nAngles);  
PHIs=[];  
for i=1:length(phi1), % nested 2 loop establishes grid of phi combinations
  for j=1:length(phi2), PHIs=[PHIs; phi1(i),phi2(j)]; end; % stack up list
end; 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations

%% set one token body pose
phiPose=pi/180*[-97 70];   % one single sample pose for drawings
figure(1); drawBody2(phiPose, Bod); % draw at one posture
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); % plot positions grey
figure(2); drawBody2(phiPose, Bod); % draw at one posture
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); % plot positions grey
figure(3); drawBody2(phiPose, Bod); % draw at one posture
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); % plot positions grey
figure(4); drawBody2(phiPose, Bod); % draw at one posture
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); % plot positions grey
figure(5); drawBody2(phiPose, Bod); % draw at one posture
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); % plot positions grey


%% Optimization params:
options.MaxIter = 1E4;                            % optimization limit
options.MaxFunEvals = 1E4;                        % optimization limit
options.nTries = 5
p0=.05*(1:(Exo.nParams*Exo.nElements*Exo.nJnts)); % INIT.GUESS (L0,r,theta)
bestCost=1e7;                                    % init very high 

%% Define some inline functions (HANDLE=@(ARGLIST)EXPRESSION constructs anon fcn & returns handle to it) 
tension = @(L0,L)    (Exo.K.*(L-L0)).*((L-L0)>0); % (inlineFcn) only +Stretch

fprintf(' parameters set. \n ')

