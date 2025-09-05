function plotMovingArm( armSide, filename, Exo, p_init)
% Animates full-body motion and plots the wrist trajectory of the selected arm

    [time, Body] = readFullBodyFile(filename);

    bodyColor = [.8 .7 .6];   % bone color
    jointColor = 'k';
    wristColor = 'g';
    torsoColor = bodyColor;
    n = size(Body.Head, 1);   % number of frames
    fields = fieldnames(Body);
    
    % --- Filter all Body fields ---
    fs = 30;
    [b, a] = butter(7, 10 / (fs / 2));

    % New processed structure
    BodyProc = struct();
    for f = 1:length(fields)
        if size(Body.(fields{f}),2) == 3
            raw = Body.(fields{f});
            validRows = ~any(isnan(raw), 2);
            if sum(validRows) > 180
                filtered = movmean(filtfilt(b, a, raw(validRows,:)), 13, 'Endpoints', 'shrink');
                Body.(fields{f}) = nan(size(raw));
                Body.(fields{f})(validRows,:) = filtered;
            end
        end
    end

    % --- Select joints based on arm side ---
    if lower(armSide) == 'l'
        shoulder = Body.ShoulderLeft;
        elbow    = Body.ElbowLeft;
        wrist    = Body.WristLeft;
    else
        shoulder = Body.ShoulderRight;
        elbow    = Body.ElbowRight;
        wrist    = Body.WristRight;
    end
    neck = Body.Neck;

    %% --------------------- Filtering positions --------------------------
    nFrames = size(shoulder, 1);
    
    % --- Extract and reshape parameters ---
    nparams = Exo.nParamsSh * round(Exo.numbconstraints(2));
    p_wr = p_init(end-2:end);
    p_el = p_init(1:nparams+1);
    p_sw = p_init(nparams+2:nparams*2+2);
    numbsh = round(p_el(1));
    numbswivel = round(p_sw(1));

    % --- Skeleton connections ---
    connections = {
        'Head', 'Neck';
        'Neck', 'SpineShoulder';
        'SpineShoulder', 'ShoulderLeft';
        'SpineShoulder', 'ShoulderRight';
        'ShoulderLeft', 'ElbowLeft';
        'ElbowLeft', 'WristLeft';
        'WristLeft', 'HandLeft';
        'HandLeft', 'HandTipLeft';
        'HandLeft', 'ThumbLeft';
        'ShoulderRight', 'ElbowRight';
        'ElbowRight', 'WristRight';
        'WristRight', 'HandRight';
        'HandRight', 'HandTipRight';
        'HandRight', 'ThumbRight';
        'SpineShoulder', 'SpineMid';
        'SpineMid', 'SpineBase';
        'SpineBase', 'HipLeft';
        'SpineBase', 'HipRight';
        'HipLeft', 'KneeLeft';
        'KneeLeft', 'AnkleLeft';
        'AnkleLeft', 'FootLeft';
        'HipRight', 'KneeRight';
        'KneeRight', 'AnkleRight';
        'AnkleRight', 'FootRight';
    };

    % --- Global axis limits ---
    allPoints = [];
    for f = 1:length(fields)
        allPoints = [allPoints; Body.(fields{f})];
    end
    padding = 0.1;
    xLimits = [min(allPoints(:,1)), max(allPoints(:,1))] + padding * [-1, 1];
    yLimits = [min(allPoints(:,2)), max(allPoints(:,2))] + padding * [-1, 1];
    zLimits = [min(allPoints(:,3)), max(allPoints(:,3))] + padding * [-1, 1];

    % --- Setup figure ---
    fig = figure('Color', 'w');
    title(['Full Body Animation: ' strrep(filename, '\', '/')]);
    axis equal; grid on; view(3);

    % --- Animation loop ---
    for i = 60:nFrames-90
        set(groot,'defaultLegendAutoUpdate','off');
        cla; hold on;

        % Plot wrist trajectory as a path
        validWrist = ~any(isnan(wrist), 2);
        traj = wrist(validWrist,:);

        if size(traj,1) > 1
              % plot3(traj(:,1), traj(:,2), traj(:,3), '-', 'Color', wristColor, 'LineWidth', 0.0015);
            scatter3(traj(:,1), traj(:,2), traj(:,3), 1.5, 'filled', 'MarkerFaceColor', wristColor);
        end

        %% Plot torso using triangle patches with spine and shoulders/hips
        spineNames = {'SpineBase', 'SpineMid', 'SpineShoulder'};
        if all(isfield(Body, spineNames)) && ...
           isfield(Body, 'ShoulderLeft') && isfield(Body, 'ShoulderRight') && ...
           isfield(Body, 'HipLeft') && isfield(Body, 'HipRight')
            SB = Body.SpineBase(i,:);
            SM = Body.SpineMid(i,:);
            SS = Body.SpineShoulder(i,:);
            shL = Body.ShoulderLeft(i,:);
            shR = Body.ShoulderRight(i,:);
            hpL = Body.HipLeft(i,:);
            hpR = Body.HipRight(i,:);

            triangles = {
                SB, hpL, hpR;
                SB, SM, hpL;
                SB, SM, hpR;
                SM, SS, shL;
                SM, SS, shR;
                SS, shL, shR;
                SM, shL, hpL;  % New triangle: shoulder, spineMid, hip
                SM, shR, hpR;
            };
            for t = 1:size(triangles,1)
                pts = cell2mat(triangles(t,:)');
                patch('Faces', [1 2 3], 'Vertices', pts, 'FaceColor', torsoColor, 'FaceAlpha', 1, 'EdgeColor', torsoColor);
            end
        end

        %% Plot joints
        for j = 1:length(fields)
            pt = Body.(fields{j})(i,:);
            if ~any(isnan(pt))
                plot3(pt(1), pt(2), pt(3), 'o', 'MarkerFaceColor', jointColor, 'MarkerEdgeColor', jointColor, 'MarkerSize', 4);
            end
        end

        % Plot connections
        for j = 1:size(connections,1)
            joint1 = connections{j,1};
            joint2 = connections{j,2};
            if isfield(Body, joint1) && isfield(Body, joint2)
                p1 = Body.(joint1)(i,:);
                p2 = Body.(joint2)(i,:);
                if ~any(isnan([p1 p2]))
                    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'Color', bodyColor, 'LineWidth', 3);
                end
            end
        end

        % ----------- Compute and plot actual pins and endpoints ----------
        dir_vec = shoulder(i,:) - neck(i,:);
        if norm(dir_vec) == 0, continue; end
        dir_vec = dir_vec / norm(dir_vec);
        R = [dir_vec; 0 -1 0; 0 0 1];
        
        % Place spring for the swivel
        for jnt = 1:numbsh
            local_vec = [p_el(4 + Exo.nParamsSh*(jnt-1)), ...
                         p_el(2 + Exo.nParamsSh*(jnt-1)) * cos(p_el(3 + Exo.nParamsSh*(jnt-1))), ...
                         p_el(2 + Exo.nParamsSh*(jnt-1)) * sin(p_el(3 + Exo.nParamsSh*(jnt-1)))];
            pin = shoulder(i,:) + local_vec * R;
            endpoint = elbow(i,:) + (shoulder(i,:) - elbow(i,:)) * p_el(6 + Exo.nParamsSh*(jnt-1));
            plot3([pin(1), endpoint(1)], [pin(2), endpoint(2)], [pin(3), endpoint(3)], 'm', 'LineWidth', 0.7);
            scatter3(endpoint(1), endpoint(2), endpoint(3), 2, 'mo' );  scatter3(pin(1), pin(2), pin(3), 2, 'mo' );  
        end
        
        % Place spring for the swivel
        for jnt = 1:numbswivel
            local_vec = [p_sw(4 + Exo.nParamsSh*(jnt-1)), ...
                         p_sw(2 + Exo.nParamsSh*(jnt-1)) * cos(p_sw(3 + Exo.nParamsSh*(jnt-1))), ...
                         p_sw(2 + Exo.nParamsSh*(jnt-1)) * sin(p_sw(3 + Exo.nParamsSh*(jnt-1)))];
            pin = shoulder(i,:) + local_vec * R;
            endpoint = elbow(i,:) + (shoulder(i,:) - elbow(i,:)) * p_sw(6 + Exo.nParamsSh*(jnt-1));
            plot3([pin(1), endpoint(1)], [pin(2), endpoint(2)], [pin(3), endpoint(3)], 'k', 'LineWidth', 0.7);
            scatter3(endpoint(1), endpoint(2), endpoint(3), 2, 'ko' );  scatter3(pin(1), pin(2), pin(3), 2, 'ko' );  
        end
        
        % Place spring for the elbow
        pin_wrist       = elbow(i,:) + (shoulder(i,:) - elbow(i,:)) * p_wr(2); 
        endpoint_wrist  = wrist(i,:);
        plot3([pin_wrist(1), endpoint_wrist(1)], [pin_wrist(2), endpoint_wrist(2)], [pin_wrist(3), endpoint_wrist(3)], 'c', 'LineWidth', 0.7);
        scatter3(endpoint_wrist(1), endpoint_wrist(2), endpoint_wrist(3), 2, 'ko' );  scatter3(pin_wrist(1), pin_wrist(2), pin_wrist(3), 2, 'co' );  

        xlim(xLimits); ylim(yLimits); zlim(zLimits);
        drawnow;
    end

    hold off;
end
