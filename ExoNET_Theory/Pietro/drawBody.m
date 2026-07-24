function h=drawBody(shPos, elPos, wrPos)
% draw just the body
% patton 2019-Jan-13
% created from the draw_system by carella

fprintf('\n Drawing ... ')

%% locations for cartoon
bodyColor=  [.8 .7 .7]; % rgb color spec for shaded body parts
head_pos =  shPos'+[0;   .2];
bas_pos =   shPos'+[0;  -.5];
nose_pos1 = shPos'+[.12; .17];
nose_pos2 = shPos'+[0;   .17];
eye_pos =   shPos'+[.06; .22];
body=[  bas_pos, head_pos, nose_pos1, eye_pos, nose_pos2, shPos', elPos', wrPos'];
h=plot(body(1,:),body(2,:), 'Color',bodyColor, 'linewidth',2);
hold on; axis equal; axis tight; axis off
ellipse(head_pos(1),head_pos(2),.08,.1,0,30,bodyColor);
scatter(eye_pos(1), eye_pos(2),10,'k','filled')

fprintf('\n done Drawing.  ')

end