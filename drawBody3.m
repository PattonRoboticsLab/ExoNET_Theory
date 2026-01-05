% draw just the body based on PHIs
% VERSIONS: 2019-Feb-11 (Patton) created from the drawBody

function h=drawBody2(phis,Bod)

fprintf('\n Drawing a Pose ... ')
% subplot(1,2,1)

%% locations for cartoon
myColor=  [.85 .75 .65]; % rgb color spec for shaded body parts
Colors=[.5 .7 .95; .1 .95 .2;  .95 .6 .3].*.75; % distinct rgb color specs  
Colors=[.6 .8 .95; .5 .95 .8;  .95 .8 .5].*.75; % distinct rgb color specs  


if ~exist('phis','var'),
  phis=pi/180*[-100 70];
  Bod.M = 70;                   % body mass (kg)
  Bod.H = 6*12*.0254;           % body height
  Bod.L = [.35 .26;];           % segment lengths (humerous, arm)
  Bod.R = Bod.L.*[.45 .5];      % proximal to centers of mass of segments
end % one single sample pose for drawings

% HAT=[-.25 0; -.18 .1; -.08 0; 0 0; 1 0]; % head and trunk
humerous=[ 0  0; 1 0]; % 
forearm =[ 0  0; 1 0]; % 
elbow=[Bod.L(1)*cos(phis(1)) Bod.L(1)*sin(phis(1))];  % elbow pos
wrist=[elbow(1)+Bod.L(2)*cos(phis(1)+phis(2)), ...   % wrist pos
       elbow(2)+Bod.L(2)*sin(phis(1)+phis(2)) ];

[h,HAT]=hdrload('digitizedMan.txd');  HAT=HAT.*Bod.H;
[h,humerous]=hdrload('humerous.txd'); humerous=humerous.*Bod.L(1);
[h,forearm]=hdrload('forearm.txd');   forearm=forearm.*Bod.L(2);


% rot=[cos(q(1)) -sin(q(1)); sin(q(1)) cos(q(1))]; % rotation matrix 4 HAT
% HAT=(rot*HAT')';
HAT=T(HAT,-pi/2);
humerous=T(humerous,phis(1));
forearm=T(forearm,phis(1)+phis(2))+elbow;

scatter(0,0,5,'k','filled'); 
hold on; axis image; axis off
scatter(elbow(1), elbow(2),5,'k','filled')
scatter(wrist(1), wrist(2),5,'k','filled')
% plot(HAT(:,1),HAT(:,2),  'Color',Colors(1,:),'linewidth',2)
% plot(humerous(:,1),humerous(:,2),'Color',Colors(2,:),'linewidth',2)
% plot(forearm(:,1),forearm(:,2),'Color',Colors(3,:),'linewidth',2)

patch(HAT(:,1),HAT(:,2), Colors(1,:),'EdgeColor','None');
patch(humerous(:,1),humerous(:,2),Colors(2,:),'EdgeColor','None')
patch(forearm(:,1),forearm(:,2),Colors(3,:),'EdgeColor','None')

fprintf('done. \n  ')
end % END function

%function for trasforming
function Pnew=T(Pold,phi)
R=[cos(phi) -sin(phi); sin(phi) cos(phi)]; % rotation matrix 4 HAT
Pnew=(R*Pold')';
end % END function
