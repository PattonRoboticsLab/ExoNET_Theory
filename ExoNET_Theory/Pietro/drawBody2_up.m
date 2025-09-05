% draw just the body based on PHIs
% VERSIONS: 2019-Feb-11 (Patton) created from the drawBody
% VERSIONS: 11/09/2024 (Bonato) created from drawBody2

function h=drawBody2up(phis,Bod)

fprintf('\n Drawing a Pose ... ')

%% locations for cartoon
bodyColor =  [.8 .7 .6]; % rgb color spec for shaded body parts
shoulder = [0 0]'; %x and y
head_pos =  shoulder + [-0.1; 0];
nose_pos1 = head_pos + [0; .1];
% Draw the arm using body positions
elbow = [0.08, 0.047]'; 
wrist = [0.15 , 0.14]';
body = [shoulder, elbow, wrist]; %from upper view you can see only the head and shoulder

%% Plot the body

hold on; axis image; %axis off
ellipse( head_pos(1), head_pos(2), .08, .1, 0, 30, bodyColor );
ellipse( shoulder(1), shoulder(2), .04, .04, 0, 30, bodyColor );
ellipse( nose_pos1(1), nose_pos1(2), .02, .05, 0, 30, bodyColor );

h=plot(body(1,:),body(2,:), 'Color',bodyColor, 'linewidth',15);

scatter( 0,0,5,'k','filled')
scatter( elbow(1), elbow(2), 5, 'k','filled')
scatter( wrist(1), wrist(2), 5, 'k','filled')

fprintf('\n done Drawing.  ')

end