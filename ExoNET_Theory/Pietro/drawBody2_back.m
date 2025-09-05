% draw just the body based on PHIs
% VERSIONS: 2019-Feb-11 (Patton) created from the drawBody

function h=drawBody2_back(phis,Bod)

fprintf('\n Drawing a Pose ... ')

%% locations for cartoon
bodyColor=  [.8 .7 .6]; % rgb color spec for shaded body parts
shoulder=[0 0]';
neck_pos = shoulder +[-0.05; 0];
head_pos =  shoulder + [-0.05; .2];
bas_pos =   shoulder + [-0.05; -0.5];

elbow=[ 0.1; -0.25];  % elbow pos
wrist=[ 0.3; -0.39];

body=[bas_pos head_pos neck_pos shoulder elbow wrist];

%% Plot the arm and the body
h=plot( body(1,:), body(2,:), 'Color',bodyColor, 'linewidth',15);
hold on; axis image; %axis off
ellipse( head_pos(1), head_pos(2), .08, .1, 0, 30, bodyColor);
scatter( 0,0,5,'k','filled')
scatter(shoulder(1), shoulder(2),5,'k','filled')
scatter( elbow(1), elbow(2),5,'k','filled')
scatter( wrist(1), wrist(2),5,'k','filled')

fprintf('\n done Drawing.  ')

end