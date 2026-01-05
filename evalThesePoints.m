% evaluate exonet at thee points 
% Used to create additional outcomes not at the opto testpoints
% VERSIONS:     Patton initiated 2025-10-9
%% ~~ BEGIN SCRIPT: ~~
fprintf('\n Extra points..')
% global Exo Bod PHIs TAUsDesired tension 
% plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');    % desired- see it again 

%% set full span of posture evaluation points (angles)
nAngles = 7; % # shoulder & elbow angles in a span for evaluation
phi1=pi/180*linspace(-100,0,nAngles); phi2=pi/180*linspace(30,145,nAngles);  
PHIs=[];  
for i=1:length(phi1), % nested 2 loop establishes grid of phi combinations
  for j=1:length(phi2), PHIs=[PHIs; phi1(i),phi2(j)]; end; % stack up list
end; 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations

%% eval torques
TAUs=exoNetTorques(p,PHIs);                   % solution calc

%% final plot
plotVectField(PHIs,Bod,Pos,TAUs,'b');           % exoNET output @ points 
drawnow

fprintf(' done. \n ')

