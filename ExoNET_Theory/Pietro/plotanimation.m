function plotanimation(wrist, elbow, Pos)
    
    % check sizes are correct
    if size(wrist, 2) ~= 3 || size(elbow, 2) ~= 3
        error('Le posizioni del polso e del gomito devono essere matrici Nx3.');
    end
    
    % Number of frames to enter the cycle
    num_frames = size(wrist, 1);
    bodyColor = [.8 .7 .6]; % rgb color spec for shaded body parts

    % Create a figure for the animation
    figure; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Movement of the arm'); view(30, 180);
    
    % Set axis limit
    xlim( [ min(min([wrist(:,1); elbow(:,1)]))-0.2, max(max([wrist(:,1); elbow(:,1)]))+0.2 ] );
    ylim( [ min(min([wrist(:,2); elbow(:,2)]))-0.2, max(max([wrist(:,2); elbow(:,2)]))+0.2 ] );
    zlim( [ min(min([wrist(:,3); elbow(:,3)]))-0.2, max(max([wrist(:,3); elbow(:,3)]))+0.2 ] );
   
    % Iteration loop
    for i = 1:num_frames
        cla;
        
        % Draw the arm
        plot3([Pos.sh(1),  elbow(i,1)], [Pos.sh(2),  elbow(i,2)], [Pos.sh(3),  elbow(i,3)], 'Color', bodyColor, 'LineWidth', 3); % Shoulder to elbow
        plot3([elbow(i,1), wrist(i,1)], [elbow(i,2), wrist(i,2)], [elbow(i,3), wrist(i,3)], 'Color', bodyColor, 'LineWidth', 3); % Elbow to wrist
        
        % Draw the points
        plot3(elbow(i,1), elbow(i,2), elbow(i,3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Elbow
        plot3(wrist(i,1), wrist(i,2), wrist(i,3), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Wrist
        
        % Draw the offsets
        plot3([ elbow(i,1), Pos.gapel(i,1) ], [ elbow(i,2), Pos.gapel(i,2) ], [ elbow(i,3), Pos.gapel(i,3) ], 'r', 'LineWidth', 5); % Elbow       
        plot3([ elbow(i,1), Pos.gapsw(i,1) ], [ elbow(i,2), Pos.gapsw(i,2) ], [ elbow(i,3), Pos.gapsw(i,3) ], 'b', 'LineWidth', 5); % Wrist
        
        pause(0.4); view(3); % Pause to create the animation effect and view 3D
    end
end