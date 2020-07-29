%% SetUp.m
%setup as many paramters as possible for the stacked MARIONET applications

%% Begin Program
global Exo Bod PHIs tension TAUsDesired 

%put_fig(2,.35,.001,.25,.94); subplot(2,1,1) %clear and set display
put_fig(1,.25,.001,.25,.94); subplot(2,1,1) %clear and set display

%% ExoNETs
fprintf('\n Set ExoNets...')
Exo.K = 25; %spring stiffness
Exo.nParams = 2; % number of paramteres governing each element
Exo.nJnts = 3;   % shoulder and elbow and should elbow
Exo.nElements = 1; % number of stacked elements per joint

%% Bod
Bod.M = 70; % body mass
Bod.L = [0.35 0.26]; % segment lengths (humerous, arm)
Bod.R = Bod.L.*[.45 .5]; %proximal to centers of mass of segments

%% set span of posture evaluation points (angles)
nAngles = 7; % # shoulder & elbow angles in a span for evaluation
phi1 = pi/180*linspace(-100,0,nAngles); 
phi2 = pi/180*linspace(25,145,nAngles);
PHIs = [];

for i = 1:length(phi1) %nested 2 loop establishes grid of phi combinations
    for j = 1:length(phi2)
        PHIs = [PHIs; phi1(i),phi2(j)]; 
    end;
end;

Pos = findPos(PHIs, Bod); % positions associated with these angle combinations

%% draw at one posture
iShow = 3; %iShow = round(size(PHIs,1)*rand(1)); %pick index - a random point to draw
% figure(2); drawBody(Pos.sh(iShow,:), Pos.el(iShow,:),Pos.wr(iShow,:));
% plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); %plot positions gray
figure(1); drawBody(Pos.sh(iShow,:), Pos.el(iShow,:), Pos.wr(iShow,:));
plot(Pos.wr(:,1),Pos.wr(:,2),'.','color',.8*[1 1 1]); % plot positions grey

%% Optimiation Parameters:
options.MaxIter = 1e4; % optimization limit
options.MaxFunEvals = 1e4; %optimization limit
options.nTries = 10; % random initial restarts
%p0 = 0.05*(1:(Exo.nParams*Exo.nElements*Exo.nJnts)); %Initial.guess (L0,'r',theta)
a = pi;
b = -pi;
c = 1.5;
d = 0.5;
     for i = 1:3
         z(i) = (a-b).*rand(1,1);
         y(i) = (c-d).*rand(1,1);
         p0(i*2-1) = z(i);
         p0(i*2) = y(i);
     end

bestCost = 1e3; % initial very high

%% Define some inline functions (HANDLE = @ (ARGLIST) EXPRESSION constructs anon fcn & returns handle to it)
tension = @(L0, L) (Exo.K.*(L-L0).*((L-L0)>0)); % only positive stretch

fprintf(' parameters set. \n ')
