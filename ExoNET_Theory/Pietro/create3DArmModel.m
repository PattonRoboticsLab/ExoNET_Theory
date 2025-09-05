function robot = create3DArmModel(upperarm_length, forearm_length, Bod)
    %% Create robot
    robot = rigidBodyTree('DataFormat','row');
    robot.Gravity = [0, 0, -9.81];

    %% Segment properties taken from shoulder simulation I already have, see
    % SetUp3D
    L1 = upperarm_length;  % Upper arm
    L2 = forearm_length;   % Forearm
    m1 = Bod.weights(1);   m2 = Bod.weights(2);
    cm1_ratio = Bod.R(1);  cm2_ratio = Bod.R(2);

    %% SHOULDER YAW (Abduction/Adduction)
    body1 = rigidBody('shoulder_abduction');
    jnt1 = rigidBodyJoint('jnt1','revolute');
    jnt1.JointAxis = [0 0 1];
    setFixedTransform(jnt1, trvec2tform([0 0 0]));
    body1.Joint = jnt1;
    addBody(robot, body1, 'base');

    %% SHOULDER PITCH (Elevation)
    body2 = rigidBody('humeral_elevation');
    jnt2 = rigidBodyJoint('jnt2','revolute');
    jnt2.JointAxis = [0 1 0];
    setFixedTransform(jnt2, trvec2tform([0 0 0]));
    body2.Joint = jnt2;
    addBody(robot, body2, 'shoulder_abduction');

    %% SHOULDER AXIAL ROTATION (Pronation/Supination for whole arm)
    body3 = rigidBody('humeral_rotation');
    jnt3 = rigidBodyJoint('jnt3','revolute');
    jnt3.JointAxis = [0 0 1];  % rotate along humerus (X)
    setFixedTransform(jnt3, trvec2tform([0 0 0]));
    body3.Joint = jnt3;
    addBody(robot, body3, 'humeral_elevation');

    %% UPPER ARM with ELBOW
    body4 = rigidBody('upper_arm');
    jnt4 = rigidBodyJoint('elbow','revolute');
    jnt4.JointAxis = [0 1 0];
    setFixedTransform(jnt4, trvec2tform([0 0 L1]));
    body4.Joint = jnt4;
    body4.Mass = m1;
    body4.CenterOfMass = [0 0 cm1_ratio * L1];
    addBody(robot, body4, 'humeral_rotation');

    %% FOREARM (fixed after elbow)
    body5 = rigidBody('forearm');
    jnt5 = rigidBodyJoint('fixed','fixed');
    setFixedTransform(jnt5, trvec2tform([0 0 L2]));
    body5.Joint = jnt5;
    body5.Mass = m2;
    body5.CenterOfMass = [0 0 cm2_ratio * L2];
    addBody(robot, body5, 'upper_arm');

    %% FIGURES TO PLOT THE RESULTS
    % figure('Name','3D Arm Model 1');
    % q_example1 = [pi/4, 3*pi/4, 0, 0];
    % show(robot, q_example1, 'Frames','on', 'Visuals','on');  % <-- changed to 'on'
    % view(3); axis equal;
    % title('3D Arm Model - Pronated');
    % 
    % figure('Name','3D Arm Model 2');
    % q_example2 = [pi/4, 0, 0, 0];
    % show(robot, q_example2, 'Frames','on', 'Visuals','on');  % <-- changed to 'on'
    % view(3); axis equal;
    % title('3D Arm Model - Neutral');
    % 
    % figure('Name','3D Arm Model 3');
    % q_example2 = [pi/4, 3*pi/4, 0, 0];
    % show(robot, q_example2, 'Frames','on', 'Visuals','on');  % <-- changed to 'on'
    % view(3); axis equal;
    % title('3D Arm Model - Neutral');


    % figure("Name", "Interactive GUI");
    % gui = interactiveRigidBodyTree(robot, "MarkerScaleFactor", 0.25);
    % 
    % % Impostazione delle configurazioni iniziali
    % q_example = [pi/4, 3*pi/4, pi/2, pi/3];
    % gui.Configuration = q_example;
    % 
    % % Visualizzazione del robot
    % figure('Name','3D Arm Model'); 
    % set(gcf, 'Color', 'w');
    % q_example = [pi/4, 3*pi/4, pi/2, pi/3];
    % show(robot, q_example, 'Frames','on', 'Visuals','on');  
    % view(3); 
    % axis equal;
    % title('Arm Model RigidBodyTree'); 
    % axis off; 
    % grid off;
    % 
    % % Aggiunta di linee e annotazioni per i DOF
    % hold on;
    % % Spalla - Abduction/Adduction
    % plot3([0, 0], [0, 0], [0, 1], 'r', 'LineWidth', 2);
    % text(0, 0, 1, 'Abduction/Adduction', 'Color', 'r', 'FontSize', 10);
    % 
    % % Spalla - Elevation
    % plot3([0, 1], [0, 0], [0, 0], 'g', 'LineWidth', 2);
    % text(1, 0, 0, 'Elevation', 'Color', 'g', 'FontSize', 10);
    % 
    % % Spalla - Rotation
    % plot3([0, 0], [0, 1], [0, 0], 'b', 'LineWidth', 2);
    % text(0, 1, 0, 'Rotation', 'Color', 'b', 'FontSize', 10);
    % 
    % % Gomito
    % plot3([0, 1], [0, 0], [1, 1], 'm', 'LineWidth', 2);
    % text(1, 0, 1, 'Elbow', 'Color', 'm', 'FontSize', 10);
    % 
    % hold off;
    % 
    % % Salvataggio della figura come immagine
    % saveas(gcf, '3D_Arm_Model_Annotated.png');


    % figure('Name','3D Arm Model');
    % q_example = [0, 0,pi/2,0];  % [yaw, pitch, roll, elbow]
    % view(3); axis equal;
    % show(robot, q_example, 'Frames','on');
    % 
    % figure('Name','3D Arm Model');
    % q_example = [0, 0, 0, pi/2];  % [yaw, pitch, roll, elbow]
    % view(3); axis equal;
    % show(robot, q_example, 'Frames','on');
 
end
