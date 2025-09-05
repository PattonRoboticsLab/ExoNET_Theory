function [Pos, Rot_matrix, Angles] = SetUp_Evaluation(Bod, Pos)
% Function to evaluate the torque on the shoulder using 70° abduction, 40° h. adduction, 90°
% elbow flexion
Angles=struct();

% Define the angles in radians
Angles.abduction_angle = deg2rad(340);               % 70° abd (phi1)
Angles.horizontal_adduction_angle = deg2rad(40);     % 40° h. add (phi3)
angles_flexion=10;
Angles.flexion_angles = linspace(deg2rad(90+10^-6), deg2rad(10^-6), angles_flexion);  % 0 to 140 fl. elb.

Pos.sh = [0, 0, 0];

% Position of the elbow is always fixed
Pos.EvalEl = Bod.L(1) * [ cos(Angles.horizontal_adduction_angle) * cos(Angles.abduction_angle), sin(Angles.horizontal_adduction_angle) * cos(Angles.abduction_angle), sin(Angles.abduction_angle) ];
Pos.EvalCM1 = Bod.R(1) * [ cos(Angles.horizontal_adduction_angle) * cos(Angles.abduction_angle), sin(Angles.horizontal_adduction_angle) * cos(Angles.abduction_angle), sin(Angles.abduction_angle) ];

axis_unit = Pos.EvalEl / norm(Pos.EvalEl); % Axis unit

% Calculate the normal vector of the plane in which the forearm moves
perp_unit = cross([0,0,1] , axis_unit) / norm(cross([0,0,1], axis_unit));   % Cross product with x-axis to get the normal vector

circle_unit = cross(  axis_unit, perp_unit ) / norm( cross( axis_unit, perp_unit  ));

Rot_matrix = [  axis_unit', perp_unit', circle_unit' ]; % Rotation matrix [ x y z]

% Calculate the position of the wrist for each flexion angle
for i = 1:length(Angles.flexion_angles)

    WrPos = Bod.L(2) * [ cos(Angles.flexion_angles(i)), sin(Angles.flexion_angles(i)), 0 ];  
    Pos.EvalWr(i, :) = Pos.EvalEl + ( Rot_matrix * WrPos')'; % Position wrist

    WrCM2 = Bod.R(2) * [ cos(Angles.flexion_angles(i)), sin(Angles.flexion_angles(i)), 0 ];
    Pos.EvalCM2(i,:) = Pos.EvalEl + ( Rot_matrix * WrCM2')'; % Position CM forearm
    
end 

%% Plot the figure to see if correct
figure;
plot3(Pos.EvalWr(:,1), Pos.EvalWr(:,2), Pos.EvalWr(:,3), 'b-', 'DisplayName', 'Wrist Path');
hold on;
scatter3(Pos.EvalWr(1,1), Pos.EvalWr(1,2), Pos.EvalWr(1,3));
scatter3(Pos.EvalEl(1), Pos.EvalEl(2), Pos.EvalEl(3), 'r', 'filled', 'DisplayName', 'Elbow');
scatter3(Pos.sh(1), Pos.sh(2), Pos.sh(3), 'g', 'filled', 'DisplayName', 'Shoulder');
xlabel('X'); ylabel('Y'); zlabel('Z'); legend;
grid on; axis equal
title('Wrist Path in 3D Space');
hold off;

end