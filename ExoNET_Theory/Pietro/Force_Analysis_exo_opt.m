function Force_Analysis_exo_opt(p, Exo, Bod, Pos, Torque_Exo, Torque_desired, Force_vector)

    nparams = (length(p) - 3)/2 - 1; %Exo.nParamsSh * round(Exo.numbconstraints(2));
    indexswivel = nparams + 1;

    p_wr = p(end-2:end);   p_el = p(1:nparams+1);   p_sw = p(nparams+2:nparams*2+2);
    
    numbsh = round( p_el(1) );    numbswivel = round( p_sw(1) );
    
    Lrange = linspace( 0, Bod.L(1) + Bod.L(2), 50 );
        
    %% Plot p vector
    fprintf(' \nParameters best solution: \n')
    fprintf('\n                   R    Theta     Y      L0   Perc     K    K_rec  L0_rec L_pre_rec')
    for i = 1:round(p( 1 ))
        fprintf('\n P_elevation %g = ', i); fprintf('%.3f  ', p( Exo.nParamsSh*(i-1)+2 : Exo.nParamsSh*i+1 )' ); 
    end
    for i = 1:round(p( nparams+2 ))
        fprintf('\n P_swivel %g    = ', i); fprintf('%.3f  ', p( Exo.nParamsSh*(i-1)+2 + indexswivel : Exo.nParamsSh*i+1 + indexswivel)' ); 
    end
    fprintf('\n                   L0_wr  Perc_wr  K_wr  ')
    fprintf('\n P_elbow       = '); fprintf('%.3f   ', p( end-2:end)' ); 

    %% Plot the torque for the shoulder for first plane of elevation (0°)
    [Tau_desired_sh_firstplane, desidx_1] = sort(vecnorm( Torque_desired.shoulder(1:2:end,:), 2, 2 ));
                    Tau_exo_sh_firstplane = vecnorm( Torque_Exo.shoulder(1:2:end,:), 2, 2 );
       
        Tau_exo_sh_firstplane = Tau_exo_sh_firstplane(desidx_1);
    
    timetorque = 0:length(Tau_desired_sh_firstplane)-1;  % tempo discreto, stesso numero di campioni
    
    figure('Name', 'Torque Comparison 0° internal rotation / 0° elbow flexion');
    scatter(timetorque, vecnorm(Tau_exo_sh_firstplane, 2, 2),     'b'); hold on;
    scatter(timetorque, vecnorm(Tau_desired_sh_firstplane, 2, 2), 'r', 'filled');
    xlabel('Frames'); ylabel('Torque (Nm)'); legend('Exoskeleton Torque', 'Desired Shoulder Torqsue'); grid on;
    
    %% Plot the torque for the shoulder for second plane of elevation (90°)
    [Tau_desired_sh_secondplane, desidx_2] = sort(vecnorm( Torque_desired.shoulder(2:2:end,:), 2, 2 ));
                    Tau_exo_sh_secondplane = vecnorm(  Torque_Exo.shoulder(2:2:end,:), 2, 2 );
    
        Tau_exo_sh_secondplane = Tau_exo_sh_secondplane(desidx_2);
    
    timetorque = 0:length(Tau_desired_sh_secondplane)-1;  % tempo discreto, stesso numero di campioni
    
    figure('Name', 'Torque Comparison 90° internal rotation / 90° elbow flexion');
    scatter(timetorque, vecnorm(Tau_exo_sh_secondplane, 2, 2),     'b'); hold on;
    scatter(timetorque, vecnorm(Tau_desired_sh_secondplane, 2, 2), 'r', 'filled');
    xlabel('Frames'); ylabel('Torque (Nm)'); legend('Exoskeleton Torque', 'Desired Shoulder Torque'); grid on;
    
    %% Spring elements for the elevation
    figure('Name', 'Force vs Length - Elevation');
    threshold_forces_elevation = zeros(numbsh, 1);
    
    for i = 1:numbsh
        subplot(ceil(numbsh/2), 2, i); hold on; grid on;
        xlabel('Spring Length Tdist (m)'); ylabel('Force T (N)'); 
        k = p_el(7 + Exo.nParamsSh*(i-1)) * Exo.K;
        title(sprintf('Elevation Spring k = %.1f N/m', k));
    
        % Extract parameters for spring i
        l0 = (Bod.L(1) + p_el(2 + Exo.nParamsSh*(i-1))) / Exo.stretch_ratio_limit + ...
            ((Bod.L(1) + p_el(2 + Exo.nParamsSh*(i-1))) / Exo.stretch_ratio_limit) * p_el(5 + Exo.nParamsSh*(i-1));
        
                    l0_recoil = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_el(9 + Exo.nParamsSh*(i-1));
        L_pre_extended_recoil = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_el(10 + Exo.nParamsSh*(i-1));
        
        k_recoil = p_el(8 + Exo.nParamsSh*(i-1)) * Exo.K;
        
        if l0_recoil <= L_pre_extended_recoil
            threshold_forces_elevation(i) = - k_recoil * (l0_recoil - L_pre_extended_recoil); % Evaluate the minimun force to have the recoil system activated                
        else
            threshold_forces_elevation(i) = 0;
        end

        threshold_exceeded_points = [];  force = zeros(size(Lrange));  
    
        for j = 1:length(Lrange)
            Tdist = Lrange(j);  
            T = 0;
            pre_extension = l0 - (Exo.L_rail - L_pre_extended_recoil) - Tdist;
            
            if pre_extension < 0
                T = - k * pre_extension;
                if l0_recoil <= L_pre_extended_recoil 
                    
                    if T > threshold_forces_elevation(i)
                        k_eq = (k * k_recoil) / (k + k_recoil);
                        T = - k_eq * (l0 + l0_recoil - Tdist - Exo.L_rail);
                    end
            
                end
            end
            force(j) = T;
            
            % Find points where force exceeds threshold
            if threshold_forces_elevation(i) > 0 && T > threshold_forces_elevation(i) 
                threshold_exceeded_points = [threshold_exceeded_points, j];
            end
        
        end
        plot(Lrange, force, 'b-', 'LineWidth', 2);
        
       % Display threshold line if exists
        if ~isempty(threshold_exceeded_points)
            yline(threshold_forces_elevation(i), 'r--', 'LineWidth', 2, 'Label', sprintf('Threshold: %.2f N', threshold_forces_elevation(i)));
                    % Highlight points where force exceeds threshold
            if ~isempty(threshold_exceeded_points)
                plot(Lrange(threshold_exceeded_points), force(threshold_exceeded_points), ...
                     'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'red', 'DisplayName', 'Above Threshold');
            end

            legend('Tension spring', 'Threshold', 'Recoil system in use');
        else
            legend('Tension spring');
        end
    end
            
    %% Spring elements for the swivel
    figure('Name', 'Force vs Length - Swivel');
    threshold_forces_swivel = zeros(numbswivel, 1);

    for i = 1:numbswivel
        subplot(ceil(numbswivel/2), 2, i); hold on; grid on; xlabel('Spring Length Tdist (m)'); 
        k = p_sw(7 + Exo.nParamsSh*(i-1)) * Exo.K;
        ylabel('Force T (N)'); title(sprintf('Swivel Spring k = %.1f N/m', k));

        % Extract parameters for spring i
        l0 = (Bod.L(1) + p_sw(2 + Exo.nParamsSh*(i-1))) / Exo.stretch_ratio_limit + ...
            ((Bod.L(1) + p_sw(2 + Exo.nParamsSh*(i-1))) / Exo.stretch_ratio_limit) * p_sw(5 + Exo.nParamsSh*(i-1));
                    
                    l0_recoil = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_sw(9 + Exo.nParamsSh*(i-1));
        L_pre_extended_recoil = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_sw(10 + Exo.nParamsSh*(i-1));
               
               
        k_recoil = p_sw(8 + Exo.nParamsSh*(i-1)) * Exo.K;
        
        force = zeros(size(Lrange));
        if l0_recoil <= L_pre_extended_recoil
            threshold_forces_swivel(i) = - k_recoil * (l0_recoil - L_pre_extended_recoil);
        else
            threshold_forces_swivel(i) = 0;
        end

        threshold_exceeded_points = [];
        for j = 1:length(Lrange)
            Tdist = Lrange(j);  
            T = 0;
            pre_extension = l0 - (Exo.L_rail - L_pre_extended_recoil) - Tdist;
            
            if pre_extension < 0
                T = - k * pre_extension;
                
                if l0_recoil <= L_pre_extended_recoil                 
                    
                    if T > threshold_forces_swivel(i)
                        k_eq = (k * k_recoil) / (k + k_recoil);
                        T = - k_eq * (l0 + l0_recoil - Tdist - Exo.L_rail);
                    end

                end
            end
            force(j) = T;
            
            % Find points where force exceeds threshold
            if threshold_forces_swivel(i) > 0 && T > threshold_forces_swivel(i)
                threshold_exceeded_points = [threshold_exceeded_points, j];
            end
        
        end        
        plot(Lrange, force, 'b-', 'LineWidth', 2);
        
        % Display threshold line
        if ~isempty(threshold_exceeded_points)

            yline(threshold_forces_swivel(i), 'r--', 'LineWidth', 2, 'Label', sprintf('Threshold: %.2f N', threshold_forces_swivel(i)));
                    % Highlight points where force exceeds threshold

            if ~isempty(threshold_exceeded_points)
                plot(Lrange(threshold_exceeded_points), force(threshold_exceeded_points), ...
                     'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'red', 'DisplayName', 'Above Threshold');
            end
            
            legend('Tension spring', 'Threshold', 'Recoil system in use');
        else
            legend('Tension spring');
        end
    end
    
    %% Spring elements on the wrist
    figure('Name', 'Force vs Length - Wrist');
    xlabel('Spring Length Tdist (m)'); ylabel('Force T (N)'); 
    k = p_wr(2) * Exo.K;
    title(sprintf('Wrist Spring k = %.1f N/m', k)); hold on; grid on;
    
    l0 = (Bod.L(2) + Bod.L(1)) / Exo.stretch_ratio_limit + (Bod.L(2) + Bod.L(1)) * p_wr(1);

    force = zeros(size(Lrange));
    for j = 1:length(Lrange)
        Tdist = Lrange(j);
        pre_extension = l0 - Tdist;
        if pre_extension < 0
            force(j) = k * pre_extension;
        else
            force(j) = 0;
        end
    end
    plot(Lrange, force, 'b-', 'LineWidth', 2);
    
    %% Display threshold forces in command window
    fprintf('\n\n=== THRESHOLD FORCES SUMMARY ===\n');
    fprintf('Elevation Springs:\n');
    for i = 1:numbsh
        fprintf('  Spring %d: %.3f N\n', i, threshold_forces_elevation(i));
    end
    fprintf('\nSwivel Springs:\n');
    for i = 1:numbswivel
        fprintf('  Spring %d: %.3f N\n', i, threshold_forces_swivel(i));
    end
    fprintf('\n');
    
    %% ============== Find which position use recoil system ===============
           
    %% Set the pins, L0 and endpoint
          numbsh = round( p_el(1) );                        numbswivel = round( p_sw(1) );    
        endpoint = zeros(size(Pos.elbowSwivel,1), 3);       endpointsw = zeros(size(Pos.elbowSwivel,1), 3); 
    l0_elevation = zeros(1, numbsh);                         l0_swivel = zeros(1, numbswivel);
    l0_recoil_el = zeros(1, numbsh);                      l0_recoil_sw = zeros(1, numbswivel);
   L0_pre_ext_el = zeros(1, numbsh);                     L0_pre_ext_sw = zeros(1, numbswivel);
     L_elevation = zeros(size(Pos.elbowSwivel,1), numbsh);    L_swivel = zeros(size(Pos.elbowSwivel,1), numbswivel );
 Actual_Pin_elevation = zeros(numbsh,3);             Actual_Pin_swivel = zeros(numbswivel,3);  

    %% Create Pins for evaluation
    for i = 1:numbsh
        Actual_Pin_elevation(i,:) = Pos.sh + [ p_el( 4 + Exo.nParamsSh*(i-1)), ...
                                    p_el( 2 + Exo.nParamsSh*(i-1)) * cos( p_el( 3 + Exo.nParamsSh*(i-1)) ), ...
                                    p_el( 2 + Exo.nParamsSh*(i-1)) * sin( p_el( 3 + Exo.nParamsSh*(i-1)) ) ]; 
        
         l0_elevation(i) = ( Bod.L(1) + p_el( 2 + Exo.nParamsSh*(i-1) ) ) / Exo.stretch_ratio_limit + ...
                          (( Bod.L(1) + p_el( 2 + Exo.nParamsSh*(i-1) ))  / Exo.stretch_ratio_limit ) * p_el( 5 + Exo.nParamsSh*(i-1));
       
         l0_recoil_el(i) = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_el( 9 + Exo.nParamsSh*(i-1));

        L0_pre_ext_el(i) = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_el( 10 + Exo.nParamsSh*(i-1));
        
    end

    for i = 1:numbswivel   
        Actual_Pin_swivel(i,:) = Pos.sh + [ p_sw( 4 + Exo.nParamsSh*(i-1)), ...
                                 p_sw( 2 + Exo.nParamsSh*(i-1)) * cos( p_sw( 3 + Exo.nParamsSh*(i-1)) ), ...
                                 p_sw( 2 + Exo.nParamsSh*(i-1)) * sin( p_sw( 3 + Exo.nParamsSh*(i-1)) ) ];    
    
            l0_swivel(i) = ( Bod.L(1) + p_sw( 2 + Exo.nParamsSh*(i-1) ) ) / Exo.stretch_ratio_limit + ...
                          (( Bod.L(1) + p_sw( 2 + Exo.nParamsSh*(i-1) ))  / Exo.stretch_ratio_limit ) * p_sw( 5 + Exo.nParamsSh*(i-1)); 
    
         l0_recoil_sw(i) = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_sw( 9 + Exo.nParamsSh*(i-1));

        L0_pre_ext_sw(i) = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_sw( 10 + Exo.nParamsSh*(i-1));
    end     

    %% Find the torques on the shoulder given by the exo
    for j = 1:size(Pos.elbowSwivel,1) 
        for i = 1:numbsh  
               endpoint(j,:) = Pos.gapel(j,:) + ( Pos.sh - Pos.gapel(j,:) ) * p_el( 6 + Exo.nParamsSh*(i-1) ) ; % Attachment on the elbow
            L_elevation(j,i) = norm( endpoint(i,:) - Actual_Pin_elevation(i,:) );  
        end 
        for i = 1:numbswivel    
            endpointsw(j,:) = Pos.gapsw(j,:) + ( Pos.sh - Pos.gapsw(j,:) ) * p_sw( 6 + Exo.nParamsSh*(i-1) ) ; % Attachment on the elbow
              L_swivel(j,i) = norm( endpointsw(i,:) - Actual_Pin_swivel(i,:) );
        end
    end
       
    %% Find the torques on the shoulder given by the exo during FRONTAL ELEVATION
    recoil_collected_elevation = [];     recoil_collected_swivel = [];
    for j = 1:size(Pos.elbowSwivel,1)      
        for i = 1:numbsh 
                   k = p_el( 7 + Exo.nParamsSh*(i-1)) * Exo.K;
            k_recoil = p_el( 8 + Exo.nParamsSh*(i-1)) * Exo.K;

            % in order to set a threshold of force
            rVect = endpoint(j,:) - Pos.sh;  lVect = Actual_Pin_elevation(i,:) - Pos.sh;        
             Tdir = lVect - rVect;           Tdist = sqrt(sum(Tdir.^2, 2));  
                        
            pre_extension = l0_elevation(i) - (Exo.L_rail - L0_pre_ext_el(i)) - Tdist; % Evaluate if the spring is slacked
        
            if pre_extension < 0  % Spring not slacked
                T = - k * pre_extension; % Tension fo the spring 
                Tmin = - k_recoil * (l0_recoil_el(i) - L0_pre_ext_el(i)); % Evaluate the minimun force to have the recoil system activated
                    
                if l0_recoil_el(i) <= L0_pre_ext_el(i) % Recoil is pre-extended    
                    if T > Tmin
                        recoil_collected_elevation = [recoil_collected_elevation, j];
                    end
                end 
            end 
        end
        
        for i = 1:numbswivel    
                   k_sw = p_sw( 7 + Exo.nParamsSh*(i-1)) * Exo.K;
            k_recoil_sw = p_sw( 8 + Exo.nParamsSh*(i-1)) * Exo.K;

            % in order to set a threshold of force
            rVect = endpointsw(j,:) - Pos.sh;  lVect = Actual_Pin_swivel(i,:) - Pos.sh;        
             Tdir = lVect - rVect;             Tdist = sqrt(sum(Tdir.^2, 2));  
                        
            pre_extension_sw = l0_swivel(i) - (Exo.L_rail - L0_pre_ext_sw(i)) - Tdist; % Evaluate if the spring is slacked
        
            if pre_extension_sw < 0  % Spring not slacked
                   T = - k_sw * pre_extension_sw; % Tension for the spring 
                Tmin = - k_recoil_sw * (l0_recoil_sw(i) - L0_pre_ext_sw(i)); % Evaluate the minimun force to have the recoil system activated
                    
                if l0_recoil_sw(i) <= L0_pre_ext_sw(i) % Recoil is pre-extended    
                    if T > Tmin
                        recoil_collected_swivel = [recoil_collected_swivel, j];
                    end
                end       
            end  
        end       
    end
    
    wrist_positions = Pos.wrSwivel;  
    num_positions = size(wrist_positions, 1);
    
    %% =============== Plot for the elevation springs======================
    % Combine indexes and remove duplicated
    valid_indices = recoil_collected_elevation(recoil_collected_elevation <= num_positions & recoil_collected_elevation > 0);
    
    threshold_exceeded = false(num_positions, 1);
    threshold_exceeded(valid_indices) = true;
        
    % Crea il plot 3D
    figure('Name', '3D Wrist Positions - Force Threshold Analysis with Body (using recoil_collected)');
    hold on; grid on;
    % Plotta nancy_body
    nancy_body;
    
   % Plot dei punti che NON superano il threshold (blu)
    below_threshold_idx = ~threshold_exceeded;
    if any(below_threshold_idx)
        pos_below = wrist_positions(below_threshold_idx, :);
        scatter3(pos_below(:, 1), pos_below(:, 2), pos_below(:, 3), ...
                 50, 'b', 'filled', 'MarkerEdgeColor', 'k');
    end
    
    % Plot dei punti che superano il threshold (rosso)
    above_threshold_idx = threshold_exceeded;
    if any(above_threshold_idx)
        pos_above = wrist_positions(above_threshold_idx, :);
        scatter3(pos_above(:, 1), pos_above(:, 2), pos_above(:, 3), ...
                 50, 'r', 'filled', 'MarkerEdgeColor', 'k');
    end
    
    % Personalizza il plot
    xlabel('X Position (m)');  ylabel('Y Position (m)');  zlabel('Z Position (m)');
    title('3D Wrist Positions - Force Threshold Analysis Swivel');
    
    % Aggiungi legenda manualmente
    legend_entries = {};  legend_colors = {};
    if any(below_threshold_idx)
        legend_entries{end+1} = 'Below Threshold';
        legend_colors{end+1} = 'b';
    end
    
    if any(above_threshold_idx)
        legend_entries{end+1} = 'Above Threshold';
        legend_colors{end+1} = 'r';
    end
    
    % Crea elementi fittizi invisibili solo per la legenda
    if ~isempty(legend_entries)
        legend_handles_dummy = [];
        for i = 1:length(legend_entries)
            h_dummy = scatter3(NaN, NaN, NaN, 50, legend_colors{i}, 'filled', 'MarkerEdgeColor', 'k');
            legend_handles_dummy(end+1) = h_dummy;
        end
        legend(legend_handles_dummy, legend_entries, 'Location', 'best');
    end
    
    % Imposta visualizzazione 3D ottimale
    view(45, 30); axis equal;
    
    %% ===================Plot for the swivel springs =====================
    valid_indices = recoil_collected_swivel(recoil_collected_swivel <= num_positions & recoil_collected_swivel > 0);
    
    threshold_exceeded = false(num_positions, 1);
    threshold_exceeded(valid_indices) = true;
        
    % Crea il plot 3D
    figure('Name', '3D Wrist Positions - Force Threshold Analysis with Body (using recoil_collected)');
    hold on; grid on;
    % Plotta nancy_body
    nancy_body;
    
   % Plot dei punti che NON superano il threshold (blu)
    below_threshold_idx = ~threshold_exceeded;
    if any(below_threshold_idx)
        pos_below = wrist_positions(below_threshold_idx, :);
        scatter3(pos_below(:, 1), pos_below(:, 2), pos_below(:, 3), ...
                 50, 'b', 'filled', 'MarkerEdgeColor', 'k');
    end
    
    % Plot dei punti che superano il threshold (rosso)
    above_threshold_idx = threshold_exceeded;
    if any(above_threshold_idx)
        pos_above = wrist_positions(above_threshold_idx, :);
        scatter3(pos_above(:, 1), pos_above(:, 2), pos_above(:, 3), ...
                 50, 'r', 'filled', 'MarkerEdgeColor', 'k');
    end
    
    % Personalizza il plot
    xlabel('X Position (m)');  ylabel('Y Position (m)');  zlabel('Z Position (m)');
    title('3D Wrist Positions - Force Threshold Analysis Swivel');
    
    % Aggiungi legenda manualmente
    legend_entries = {};  legend_colors = {};
    if any(below_threshold_idx)
        legend_entries{end+1} = 'Below Threshold';
        legend_colors{end+1} = 'b';
    end
    
    if any(above_threshold_idx)
        legend_entries{end+1} = 'Above Threshold';
        legend_colors{end+1} = 'r';
    end
    
    % Crea elementi fittizi invisibili solo per la legenda
    if ~isempty(legend_entries)
        legend_handles_dummy = [];
        for i = 1:length(legend_entries)
            h_dummy = scatter3(NaN, NaN, NaN, 50, legend_colors{i}, 'filled', 'MarkerEdgeColor', 'k');
            legend_handles_dummy(end+1) = h_dummy;
        end
        legend(legend_handles_dummy, legend_entries, 'Location', 'best');
    end
        
    % Imposta visualizzazione 3D ottimale
    view(45, 30); axis equal;

    %% ================== Statistic information ===========================
    % Aggiungi informazioni statistiche
    num_above = sum(threshold_exceeded);   num_below = sum(~threshold_exceeded);
    percentage_above = (num_above / length(threshold_exceeded)) * 100;
    
    % Informazioni dettagliate sui vettori recoil_collected
    num_elevation_exceeded = length(recoil_collected_elevation);
    num_swivel_exceeded = length(recoil_collected_swivel);
        
    fprintf('\n=== 3D THRESHOLD ANALYSIS (using recoil_collected) ===\n');
    fprintf('Total positions analyzed: %d\n', num_positions);
    fprintf('Elevation positions exceeding threshold: %d\n', num_elevation_exceeded);
    fprintf('Swivel positions exceeding threshold: %d\n', num_swivel_exceeded);
    fprintf('Positions below threshold: %d (%.1f%%)\n', num_below, 100-percentage_above);
    
    if ~isempty(recoil_collected_elevation)
        fprintf('Elevation indices range: %d to %d\n', min(recoil_collected_elevation), max(recoil_collected_elevation));
    end
    if ~isempty(recoil_collected_swivel)
        fprintf('Swivel indices range: %d to %d\n', min(recoil_collected_swivel), max(recoil_collected_swivel));
    end
    fprintf('========================================================\n\n');

    %% Plot the forces from the simulation
    F_des_ext = vecnorm(Force_vector.desired_extended, 2, 2);   
    F_exo_ext = vecnorm(Force_vector.exo_extended, 2, 2);
    F_des_flex = vecnorm(Force_vector.desired_flexed, 2, 2);    
    F_exo_flex = vecnorm(Force_vector.exo_flexed, 2, 2);
    
    [sorted_des_ext, idx_ext] = sort(F_des_ext);    
    sorted_exo_ext = F_exo_ext(idx_ext);
    [sorted_des_flex, idx_flex] = sort(F_des_flex);  
    sorted_exo_flex = F_exo_flex(idx_flex);
    
    %% Scatter plot - Extended
    figure('Name','Force Comparison - Extended');
    scatter(1:length(sorted_des_ext), sorted_des_ext, 'r', 'filled'); hold on;
    scatter(1:length(sorted_exo_ext), sorted_exo_ext, 'b');
    legend('Desired', 'Exo');         
    xlabel('Sample Index (Sorted by Desired Force)');
    ylabel('Force Magnitude (N)');    
    title('Force Magnitude Comparison - Extended');
    grid on;
    
    %% Scatter plot - Flexed
    figure('Name','Force Comparison - Flexed');
    scatter(1:length(sorted_des_flex), sorted_des_flex, 'r', 'filled'); hold on;
    scatter(1:length(sorted_exo_flex), sorted_exo_flex, 'b');
    legend('Desired', 'Exo');          
    xlabel('Sample Index (Sorted by Desired Force)');
    ylabel('Force Magnitude (N)');     
    title('Force Magnitude Comparison - Flexed');
    grid on;

end