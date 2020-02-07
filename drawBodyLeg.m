% ***********************************************************************
% Draw the Body on the background based on the angles of phiPose
% ***********************************************************************

function h = drawBodyLeg(phiPose,BODY)

bodyColor = [0.8, 0.7, 0.6]; % RGB color space for shaded body parts

shoulder = [0; 0.5];

head_position = shoulder + [0; 0.2];
nose_position1 = shoulder + [0.1; 0.17];
nose_position2 = shoulder + [0; 0.17];
eye_position = shoulder + [0.05; 0.22];

hip = [0; 0]; % HIP position

knee = [BODY.Lengths(1)*sind(phiPose(1)); ... % KNEE position
        -(BODY.Lengths(1)*cosd(phiPose(1)))];

ankle = [knee(1) + BODY.Lengths(2)*sind(phiPose(1)-phiPose(2)); ... % ANKLE position
         knee(2) - BODY.Lengths(2)*cosd(phiPose(1)-phiPose(2))];

body = [ankle, knee, hip, shoulder, head_position, nose_position1, eye_position, nose_position2];

h = plot(body(1,:),body(2,:),'Color',bodyColor,'linewidth',15);
hold on
axis image
axis off
ellipse(head_position(1),head_position(2),0.08,0.1,0,30,bodyColor);
scatter(shoulder(1),shoulder(2),5,'k','filled')
scatter(hip(1),hip(2),10,'r','filled')
scatter(knee(1),knee(2),10,'k','filled')
scatter(ankle(1),ankle(2),10,'k','filled')
scatter(eye_position(1),eye_position(2),20,'k','filled')

end