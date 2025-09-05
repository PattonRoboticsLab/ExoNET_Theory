function compareJointMotion(Body, Bod, armSide, filename)
% Compares original 3D motion from Body data with simulated motion of 3D robot model
% Plots them side by side frame-by-frame

    shoulder = Body.Shoulder;
    elbow    = Body.Elbow;
    wrist    = Body.Wrist;
    
    if lower(armSide) == 'l'
        armLabel = 'Left';
    else
        armLabel = 'Right';
    end

    %% Remove invalid frames
    validRows = ~any(isnan([shoulder, elbow, wrist]), 2);
    shoulder = shoulder(validRows, :);
    elbow    = elbow(validRows, :);
    wrist    = wrist(validRows, :);
    n = size(shoulder, 1);
    % 
    % %% Create robot model
    % %robot = create3DArmModel(Bod);
    % 
    % %% Precompute joint angles
    % q = zeros(n, 4);
    % for i = 1:n
    %     q(i,:) = computeJointAngles3D(shoulder(i,:), elbow(i,:), wrist(i,:));
    % end

    %% Compute axis limits for original motion
    allPoints = [ shoulder; elbow; wrist ];
    x_range_real = [min(allPoints(:,1)) max(allPoints(:,1))];
    y_range_real = [min(allPoints(:,2)) max(allPoints(:,2))];
    z_range_real = [min(allPoints(:,3)) max(allPoints(:,3))];

    %% Create figure window
    fig = figure('Name', ['Original vs Simulated Arm - ' armLabel]);

    for i = 1:n
        clf(fig);

        % --- Original Arm ---
        figure();
        hold on; grid on; axis equal;

        % Traccia il segmento del braccio attuale
        plot3([shoulder(i,1), elbow(i,1)], [shoulder(i,2), elbow(i,2)], [shoulder(i,3), elbow(i,3)], 'r', 'LineWidth', 3);
        plot3([elbow(i,1), wrist(i,1)],   [elbow(i,2), wrist(i,2)],   [elbow(i,3), wrist(i,3)],   'b', 'LineWidth', 3);

        % Visualizza marker attuali
        plot3(shoulder(i,1), shoulder(i,2), shoulder(i,3), 'ko', 'MarkerFaceColor', 'k');
        plot3(elbow(i,1),    elbow(i,2),    elbow(i,3),    'ko', 'MarkerFaceColor', 'k');
        plot3(wrist(i,1),    wrist(i,2),    wrist(i,3),    'ko', 'MarkerFaceColor', 'k');

        % ✅ Aggiunta: traccia traiettoria del polso fino a frame i
        plot3(wrist(1:i,1), wrist(1:i,2), wrist(1:i,3), 'c--', 'LineWidth', 1.5);

        title('Original Motion');
        xlabel('X'); ylabel('Y'); zlabel('Z');
        xlim(x_range_real); ylim(y_range_real); zlim(z_range_real);
        view(3);

        % % --- Simulated Robot ---
        % subplot(1,2,2);
        % ax = gca;
        % try
        %     show(robot, q(i,:), 'PreservePlot', false, 'Frames','on', 'Visuals','on', 'Parent', ax);
        % catch
        %     warning('Failed to render robot at frame %d', i);
        %     continue;
        % end
        % title('Simulated Robot Motion');
        % view(ax, 3); axis(ax, 'equal'); grid(ax, 'on');
        % axis(ax, 'tight');
        % xlim(ax,[-0.5 0.5]); ylim(ax,[-0.5 0.5]); zlim(ax,[-0.5 0.5]);
        % sgtitle(['Frame ' num2str(i) ' - ' filename]);
        % drawnow;
        % pause(0.05);

   end
end