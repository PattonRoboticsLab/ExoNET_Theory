% drawExonets: draw individual exonets at arm pose specified by phis
% VERSIONS: 2019-Feb-11 (Patton) created from the drawBody2

function h=drawExonets_mod(p,phis)

%% Setup
fprintf('\n Drawing nets ... ')
global Exo Bod

%% locations for cartoon
Colors=  [.5 .7 1; .1 1 .2;  1 .6 .3]; % 3 distinct rgb color specs  
LW=4; % line widths
shoulder=[0 0];
elbow=[Bod.L(1)*cos(phis(1)) Bod.L(2)*sin(phis(1))];  % elbow pos
wrist=[elbow(1)+Bod.L(2)*cos(phis(1)+phis(2)), ...   % wrist pos
       elbow(2)+Bod.L(2)*sin(phis(1)+phis(2)) ];
% indexes-p vect stacked ea.joint, & within that ea.element, & ea.paramter:
shIndex=  1;
elIndex=  Exo.nParams*Exo.nElements+1;
shElIndex=Exo.nParams*Exo.nElements*2+1;

%% loop thorough all MARIONETs

for element=1:Exo.nElements, fprintf(' shoulder element %d..',element);
  r0=    p(shIndex+(element-1)*Exo.nParams+0);      % extract from p all the parameters
  theta0=p(shIndex+(element-1)*Exo.nParams+1);
  r1=    p(shIndex+(element-1)*Exo.nParams+2);      
  theta1=p(shIndex+(element-1)*Exo.nParams+3);
  L0=   p(shIndex+(element-1)*Exo.nParams+4);
  r0Pos=[r0*cos(theta0)       r0*sin(theta0) ]; 
  r1Pos=[r1*cos(theta1)       r1*sin(theta1) ]; % vect to rotatorPin
  elbow=[Bod.L(1)*cos(phis(1)) Bod.L(1)*sin(phis(1))];  % elbow pos
 
  plot([r0Pos(1) r1Pos(1)], [r0Pos(2) r1Pos(2)], ...
    'Color',Colors(1,:), 'linewidth',LW);
  plot([shoulder(1) r0Pos(1)], [shoulder(2) r0Pos(2)], ...
    'b', 'linewidth',LW);
  plot([shoulder(1) r1Pos(1)], [shoulder(2) r1Pos(2)], ...
    'y', 'linewidth',LW);
 
end

for element=1:Exo.nElements, fprintf(' elbow element %d..',element);
  r0=     p(elIndex+(element-1)*Exo.nParams+0);      % extract from p
  theta0= p(elIndex+(element-1)*Exo.nParams+1);
  r1=     p(elIndex+(element-1)*Exo.nParams+2);      
  theta1= p(elIndex+(element-1)*Exo.nParams+3);
  L0=    p(elIndex+(element-1)*Exo.nParams+4);
  elbow=[Bod.L(1)*cos(phis(1)) Bod.L(1)*sin(phis(1))];  % elbow pos
  r0Pos=elbow+[r0*cos(phis(1)+theta0)  r0*sin(phis(1)+theta0) ];  % vect to rotatorPin
  r1Pos=elbow+[r1*cos(phis(1)+theta1)  r1*sin(phis(1)+theta1) ];  % vect to rotatorPin
  wrist=[elbow(1)+Bod.L(2)*cos(phis(1)+phis(2)), ...   % wrist pos
         elbow(2)+Bod.L(2)*sin(phis(1)+phis(2)) ];
  
  plot([r0Pos(1) r1Pos(1)], [r0Pos(2) r1Pos(2)], ...
    'Color',Colors(2,:), 'linewidth',LW);

  
end

if Exo.nJnts==3,
  for element=1:Exo.nElements, fprintf(' 2joint element %d..',element);
    r=    p(shElIndex+(element-1)*Exo.nParams+0);   % extract from p
    theta=p(shElIndex+(element-1)*Exo.nParams+1);
    L0=   p(shElIndex+(element-1)*Exo.nParams+2);
    rPos=[r*cos(theta)       r*sin(theta) ];  % vect to rotatorPin
    wrist=[elbow(1)+Bod.L(2)*cos(phis(1)+phis(2)), ...   % wrist pos
           elbow(2)+Bod.L(2)*sin(phis(1)+phis(2)) ];
  plot([rPos(1) wrist(1)], [rPos(2) wrist(2)],...
    'Color',Colors(3,:), 'linewidth',LW);
  end
end
  
% scatter(eye_pos(1), eye_pos(2),14,'k','filled')

fprintf('\n done Drawing.  ')

end