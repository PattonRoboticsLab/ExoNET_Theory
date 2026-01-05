% set for arm exonet applications
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

fprintf('\n\n ____ ExoNET Arm ____ \n')
global Exo Bod PHIs TAUsDesired  % tension 

%% Bod
Bod.M = 70;                   % body mass (kg)
Bod.L = [.35 .26;];           % segment lengths (humerous, arm)
Bod.H = 6*12*.0254;           % body height
Bod.R = Bod.L.*[.45 .5];      % proximal to centers of mass of segments 

%% set full span of posture evaluation points (angles)
nAngles = 7; % # shoulder & elbow angles in a span for evaluation
phi1=pi/180*linspace(-100,0,nAngles); phi2=pi/180*linspace(30,145,nAngles);  
PHIs=[];  
for i=1:length(phi1), % nested 2 loop establishes grid of phi combinations
  for j=1:length(phi2), PHIs=[PHIs; phi1(i),phi2(j)]; end; % stack up list
end; 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations
pose=pi/180*[-100 70];   % one single sample pose for drawings
pose=pi/180*[-20 60];   % one single sample pose for drawings

%% set one token body pose
put_fig(1,.03,.2,.8,.7); clf; hold on; % clear and set display:
subplot(1,2,1); 
drawBody3(pose, Bod);    % draw at one posture
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); % testPoints in grey

%% general Optimization params:
options.nTries = 10;                             % #random init restarts
options.MaxIter = 1E4;                            % optimization limit
options.MaxFunEvals = 5E4                        % optimization limit

fprintf(' Basic parameters set. \n ')

