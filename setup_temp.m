% % set as many parameters as possible for the stacked marionet applications
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
    

%% MARIONETS
Exo.K=1000;         % spring Stiffness 
Exo.stretch_ratio_limit = 2; %
Exo.nParams=3;      % number of parameters governing each element
Exo.nJnts=3;        % shoulder and elbow and shoulder elbow



% set desired CONSTRIANTS on the parameters: 
RLoHi=[.05 .15];thetaLoHi=2*pi*[0 1];  L0LoHi=[.21 .50];               % ranges
i=0; Exo.pConstraint=NaN*zeros(Exo.nJnts*Exo.nElements*Exo.nParams,2); % init
for joint=1:Exo.nJnts
  for element=1:Exo.nElements
    i=i+1; Exo.pConstraint(i,:)=RLoHi;
    i=i+1; Exo.pConstraint(i,:)=thetaLoHi;
    i=i+1; Exo.pConstraint(i,:)=L0LoHi;
  end
end

%% Bod
Bod.M = 50;                   % body mass 
Bod.L = [.28 .25;];           % segment lengths (humerous, arm)
Bod.R = Bod.L.*[.45 .5];      % proximal to centers of mass of segments 
Bod.pose=pi/180*[-97 70];     % token body position (can be anything)

%% Setup span of full workspace posture evaluation points (angles)
nAngles = 10; % # shoulder & elbow angles in a span for evaluation
phi1=pi/180*linspace(-100,0,nAngles); phi2=pi/180*linspace(25,145,nAngles);  
PHIs=[];  
for i=1:length(phi1)          % nested 2-loop establishes grid of phi's
  for j=1:length(phi2), PHIs=[PHIs; phi1(i),phi2(j)]; end % stack up list
end 
Pos=forwardKin(PHIs,Bod);     % positions assoc w/ these angle combinations
PHIsWorkspace=PHIs;           % store this
PosWorkspace=Pos;             % store this

Exo.T = zeros(size(PHIs, 1), 3, Exo.nElements)
Exo.Tdist = zeros(size(PHIs, 1), 3, Exo.nElements)
size(Exo.T)
%% Optimization params:
optOptions=optimset();
optOptions.MaxIter = 1E6;                             % optimization limit
optOptions.MaxFunEvals = 1E6;                         % optimization limit
% optOptions.TolX = 1e-13;
% optOptions.TolFun = 1e-13;
optimset(optOptions);
nTries = 100;                            % number optim reruns 

%% HANDLE=@(ARGLIST)EXPRESSION constructs anon fcn & returns handle to it 

%% Tension Function for Rohith's Eighth Inch Bungee Cords
%tension = @(L0,L) (112.2*(L/L0).^5-838.3*(L/L0).^4+2494*(L/L0).^3-3689*(L/L0).^2+2717*(L/L0)-794.2).*((L/L0)>1);
%% Tension for Yaseen's Quarter Inch Bungee Cords
tension = @(L0,L) (710.5*(L/L0).^5-5442*(L/L0).^4+1.654e4*(L/L0).^3-2.495e4*(L/L0).^2+1.871e4*(L/L0)-5575).*((L/L0)>1);
%tension = @(L0,L) (Exo.K.*(L-L0)).*((L-L0)>0);   % (inlineFcn) +Stretch
%% Tension Quarter Inch Maroon_Teal Bungee
%tension = @(L0,L) (370.9*(L/L0).^5-2963*(L/L0).^4+9398*(L/L0).^3-14790*(L/L0).^2+11590*(L/L0)-3600).*((L/L0)>1);
%% plot: 
%put_fig(fieldType,.9,.35,.6,.5); subplot(1,2,1);    % place figure 
drawBody2(Bod.pose,Bod);     hold on                % draw@1 posture
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.7*[1 1 1]);% positions,grey

fprintf(' parameters set. \n ')

