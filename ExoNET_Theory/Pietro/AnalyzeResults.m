function AnalyzeResults(results)
% AnalyzeResults analyzes and plots velocity, acceleration, and torque metrics
% for healthy or stroke patients. It creates bar plots with group-specific
% coloring and fixed y-axis scaling for comparison across datasets.

    % === Create output folder ===
    baseFolder = 'analysis_result';
    outputFolder = baseFolder;
    counter = 1;
    while exist(outputFolder, 'dir')
        outputFolder = sprintf('%s_%d', baseFolder, counter);
        counter = counter + 1;
    end
    mkdir(outputFolder);

    % === Patient group detection ===
    if isfield(results, 'type') && strcmpi(results.type, 'stroke')
        group_label = 'Stroke';
        color_gravity = [0.7 0.3 0.9];   % Viola chiaro
    else
        group_label = 'Healthy';
        color_gravity = [0.4 0.6 1.0];   % Blu chiaro
    end

    ylim_max = 70;  % Fixed y-axis max for torque plots

    % === Initialize metrics ===
    velocity_upperarm = [];
    velocity_forearm  = [];
    accel_upperarm    = [];
    accel_forearm     = [];
    grav_shoulder     = [];
    grav_elbow        = [];

    % === Process each trial ===
    for i = 1:length(results.torque)
        if ~isempty(results.torque{i})
            vel = results.velocity{i};
            acc = results.acceleration{i};
            torque = results.torque{i};

            if ~isempty(vel) && ~isempty(acc) && ...
               isfield(torque, "shoulder_gravity")  && isfield(torque, "elbow_gravity")

                velocity_upperarm(end+1) = mean(vel.upperarm);
                velocity_forearm(end+1)  = mean(vel.forearm);
                accel_upperarm(end+1)    = mean(acc.upperarm);
                accel_forearm(end+1)     = mean(acc.forearm);
                grav_shoulder(end+1)     = mean(torque.shoulder_gravity);
                grav_elbow(end+1)        = mean(torque.elbow_gravity);
            end
        end
    end

   %% --- Torque Summary ---
    fig1 = figure('Name', 'Torque Summary', 'Position', [100, 100, 1200, 500]);
    mean_torques = [ mean(grav_shoulder), mean(grav_elbow) ];
    std_torques  = [ std(grav_shoulder), std(grav_elbow) ];
    labels_torque = {'Grav Shoulder', 'Grav Elbow'};
    
    b1 = bar(mean_torques, 'FaceColor', color_gravity);
    hold on;
    errorbar(1:2, mean_torques, std_torques, '.k', 'LineWidth', 1.5);
    for k = 1:2
        text(k, mean_torques(k) + 0.05 * max(mean_torques), sprintf('%.2f Nm', mean_torques(k)), ...
            'HorizontalAlignment', 'center', 'FontSize', 10);
    end
    
    xticks(1:2); xticklabels(labels_torque); ylim([0 20]); ylabel('Torque (Nm)');
    title(sprintf('Torque Summary (%s Patients)', group_label));
    grid on;
    saveas(fig1, fullfile(outputFolder, 'MeanStd_Torque.png'));

    %% --- Motion Summary (velocity + acceleration) ---
    fig2 = figure('Name', 'Motion Summary', 'Position', [100, 100, 1200, 500]);
    mean_motion = [mean(velocity_upperarm), mean(velocity_forearm), mean(accel_upperarm), mean(accel_forearm)];
    std_motion  = [std(velocity_upperarm), std(velocity_forearm), std(accel_upperarm), std(accel_forearm)];
    labels_motion = {'Vel Upperarm', 'Vel Forearm', 'Acc Upperarm', 'Acc Forearm'};

    b2 = bar(mean_motion);
    hold on;
    errorbar(1:4, mean_motion, std_motion, '.k', 'LineWidth', 1.5);
    for k = 1:4
        unit = 'm/s';
        if contains(labels_motion{k}, 'Acc')
            unit = 'm/s²';
        end
        text(k, mean_motion(k) + 0.05 * max(mean_motion), sprintf('%.2f %s', mean_motion(k), unit), ...
            'HorizontalAlignment', 'center', 'FontSize', 10);
    end
    xticks(1:4); xticklabels(labels_motion); ylim([0 7]); ylabel('Value');
    title(sprintf('Velocity & Acceleration (%s Patients)', group_label));
    grid on;
    saveas(fig2, fullfile(outputFolder, 'MeanStd_Motion.png'));

end
