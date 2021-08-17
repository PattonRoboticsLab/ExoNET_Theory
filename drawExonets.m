% drawExonets: draw individual exonets at arm pose specified by phis
% VERSIONS: 2019-Feb-11 (Patton) created from the drawBody2

function h=drawExonets(p,phis)

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
  r=    p(shIndex+(element-1)*Exo.nParams+0);      % extract from p
  theta=p(shIndex+(element-1)*Exo.nParams+1);
  L0=   p(shIndex+(element-1)*Exo.nParams+2);
  rPos=[r*cos(theta)       r*sin(theta) ];  % vect to rotatorPin
  elbow=[Bod.L(1)*cos(phis(1)) Bod.L(1)*sin(phis(1))];  % elbow pos
  plot([rPos(1) elbow(1)], [rPos(2) elbow(2)],...
    'Color',Colors(1,:), 'linewidth',LW);
end

for element=1:Exo.nElements, fprintf(' elbow element %d..',element);
  r=     p(elIndex+(element-1)*Exo.nParams+0);      % extract from p
  theta= p(elIndex+(element-1)*Exo.nParams+1);
  L0=    p(elIndex+(element-1)*Exo.nParams+2);
  elbow=[Bod.L(1)*cos(phis(1)) Bod.L(1)*sin(phis(1))];  % elbow pos
  rPos=elbow+[r*cos(phis(1)+theta)  r*sin(phis(1)+theta) ];  % vect to rotatorPin
  wrist=[elbow(1)+Bod.L(2)*cos(phis(1)+phis(2)), ...   % wrist pos
         elbow(2)+Bod.L(2)*sin(phis(1)+phis(2)) ];
  plot([rPos(1) wrist(1)], [rPos(2) wrist(2)],...
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


