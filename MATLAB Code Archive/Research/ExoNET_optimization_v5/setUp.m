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
put_fig(2,.935,.001,.75,.94); subplot(1,2,1) % clear and set display:
put_fig(1,.9925,.001,.75,.94); subplot(1,2,1) % clear and set display:

%% MARIONETS
fprintf('\n  Set marionets...')
Exo.K=25;          % spring Stiffness 
Exo.nParams=3;      % number of parameters governing each element
Exo.nJnts=3;        % shoulder and elbow and shoulder elbow 
Exo.nElements=1;     % number of stacked elements per joint

%% Bod
Bod.M = 70;                   % body mass 
Bod.L = [.35 .26;];           % segment lengths (humerous, arm)
Bod.R = Bod.L.*[.45 .5];      % proximal to centers of mass of segments 

%% set span of posture evaluation points (angles)
nAngles = 7; % # shoulder & elbow angles in a span for evaluation
phi1=pi/180*linspace(-100,0,nAngles); phi2=pi/180*linspace(25,145,nAngles);  
PHIs=[];  
for i=1:length(phi1), % nested 2 loop establishes grid of phi combinations
  for j=1:length(phi2), PHIs=[PHIs; phi1(i),phi2(j)]; end; 
end; 
Pos=findPos(PHIs,Bod);% positions assoc w/ these angle combinations

%% draw at one posture
iShow=3; %iShow=round(size(PHIs,1)*rand(1)); % pick index - a random point to draw
figure(2); drawBody(Pos.sh(iShow,:), Pos.el(iShow,:), Pos.wr(iShow,:));
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); % plot positions grey
figure(1); drawBody(Pos.sh(iShow,:), Pos.el(iShow,:), Pos.wr(iShow,:));
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); % plot positions grey

%% Optimization params:
options.MaxIter = 1E4;                            % optimization limit
options.MaxFunEvals = 1E4;                        % optimization limit
options.nTries = 104;                              % #random init restarts
p0=.05*(1:(Exo.nParams*Exo.nElements*Exo.nJnts)); % INIT.GUESS (L0,r,theta)
bestCost=1e16;                                    % init very high 

%% Define some inline functions (HANDLE=@(ARGLIST)EXPRESSION constructs anon fcn & returns handle to it) 
tension = @(L0,L)    (Exo.K.*(L-L0)).*((L-L0)>0); % only positive stretch

fprintf(' parameters set. \n ')

