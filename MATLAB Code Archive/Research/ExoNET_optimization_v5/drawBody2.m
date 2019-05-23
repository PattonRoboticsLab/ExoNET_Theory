% draw just the body based on PHIs
% VERSIONS: 2019-Feb-11 (Patton) created from the drawBody
 
function h=drawBody2(phis,Bod)
 
fprintf('\n Drawing a Pose ... ')
 
%% locations for cartoon
bodyColor=  [.8 .7 .6]; % rgb color spec for shaded body parts
shoulder=[0 0];
head_pos =  shoulder'+[0;   .2];
bas_pos =   shoulder'+[0;  -.5];
nose_pos1 = shoulder'+[.1; .17];
nose_pos2 = shoulder'+[0;   .17];
eye_pos =   shoulder'+[.05; .22];
elbow=[Bod.L(1)*cos(phis(1)) Bod.L(1)*sin(phis(1))];  % elbow pos
wrist=[elbow(1)+Bod.L(2)*cos(phis(1)+phis(2)), ...   % wrist pos
       elbow(2)+Bod.L(2)*sin(phis(1)+phis(2)) ];
body=[bas_pos head_pos nose_pos1 eye_pos nose_pos2 shoulder' elbow' wrist'];
h=plot(body(1,:),body(2,:), 'Color',bodyColor, 'linewidth',15);
hold on; axis image; axis off
ellipse(head_pos(1),head_pos(2),.08,.1,0,30,bodyColor);
scatter(0,0,5,'k','filled')
scatter(elbow(1), elbow(2),5,'k','filled')
scatter(wrist(1), wrist(2),5,'k','filled')
scatter(eye_pos(1), eye_pos(2),20,'k','filled')
 
fprintf('\n done Drawing.  ')
 
end
