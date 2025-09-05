% draw just the body based on PHIs
% VERSIONS: 2019-Feb-11 (Patton) created from the drawBody
function h = drawBody3D(Bod)
%% locations for cartoon
bodyColor = [.8 .7 .6]; % rgb color spec for shaded body parts
shoulder  = [0 0 0];
head_pos  = shoulder' + [0;   .2; 0];
bas_pos   = shoulder' + [0;  -.5; 0];
nose_pos1 = shoulder' + [.1; .17; 0];
nose_pos2 = shoulder' + [0;  .17; 0];
eye_pos   = shoulder' + [.05; .22; 0];

% Position of the elbow and wrist
elb_angle = pi/4; wrist_angle = elb_angle;  % Fix the angles for representation
elbow = [ Bod.L(1)*cos(elb_angle)*cos(elb_angle), Bod.L(1)*cos(elb_angle)*sin(elb_angle), -Bod.L(1)*sin(elb_angle), ];
wrist = [ elbow(1)+Bod.L(2)*cos(wrist_angle)*cos(elb_angle), elbow(2)+Bod.L(2)*sin(wrist_angle)*cos(elb_angle), elbow(3)+Bod.L(2)*sin(wrist_angle),];

body = [bas_pos head_pos nose_pos1 eye_pos nose_pos2 shoulder' elbow' wrist'];

%% Plot the arm and the body
% Body=[x;y;z]
%% Plot the body
plot3(body(1,1:6), body(2,1:6), body(3,1:6), 'Color', bodyColor, 'linewidth', 5); hold on;
plot3(body(1,6:8), body(2,6:8), body(3,6:8), 'Color', bodyColor, 'linewidth', 15);
hold on; axis image %axis off
scatter3(elbow(1), elbow(2), elbow(3), 5, 'k', 'r'); 
scatter3(wrist(1), wrist(2), wrist(3), 5, 'k', 'filled');
end

%% next impletation should include the visualization of the head and the whole body
%ellipse3D(head_pos(2),head_pos(3),head_pos(1),.04,.04,.1,bodyColor);
%scatter3(0,0,5,'k','filled')
%scatter3(eye_pos(1), eye_pos(2),20,'k','filled')
