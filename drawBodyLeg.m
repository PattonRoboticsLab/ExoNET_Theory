% ***********************************************************************
% Draw the Body on the background based on the angles of BODY.pose
% ***********************************************************************

function h = drawBodyLeg(BODY)

bodyColor = [0.8, 0.7, 0.6]; % RGB color space for shaded body parts

shoulder = [0; 0.5];

head_position = shoulder + [0; 0.2];
nose_position1 = shoulder + [0.09; 0.18];
nose_position2 = shoulder + [0; 0.18];
eye_position = shoulder + [0.05; 0.22];

elbow = shoulder - [0.32*cosd(70); 0.32*sind(70)];

wrist = shoulder - [elbow(1)+0.25*cosd(110); elbow(2)+0.25*sind(110)];

hip = [0; 0]; % HIP position

knee = [BODY.Lengths(1)*sind(BODY.pose(1)); ... % KNEE position
        -(BODY.Lengths(1)*cosd(BODY.pose(1)))];

ankle = [knee(1) + BODY.Lengths(2)*sind(BODY.pose(1)-BODY.pose(2)); ... % ANKLE position
         knee(2) - BODY.Lengths(2)*cosd(BODY.pose(1)-BODY.pose(2))];

body = [ankle, knee, hip, head_position, nose_position1, eye_position, nose_position2, shoulder, elbow, wrist];

h = plot(body(1,:),body(2,:),'Color',bodyColor,'linewidth',15);
hold on
axis image
axis off
ellipse(head_position(1),head_position(2),0.08,0.1,0,30,bodyColor);
scatter(shoulder(1),shoulder(2),5,'k','filled')
scatter(hip(1),hip(2),35,'k','filled')
scatter(knee(1),knee(2),35,'k','filled')
scatter(ankle(1),ankle(2),35,'k','filled')
scatter(eye_position(1),eye_position(2),20,'k','filled')

end