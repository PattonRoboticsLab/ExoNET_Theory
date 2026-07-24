function Force_Analysis(p, Exo, Bod)

    nparams = (length(p) - 2)/2 - 1; %Exo.nParamsSh * round(Exo.numbconstraints(2));
    indexswivel = nparams + 1;

    p_wr = p(end-2:end);   p_el = p(1:nparams+1);   p_sw = p(nparams+2:nparams*2+2);    
    numbsh = round( p_el(1) );    numbsw = round( p_sw(1) );

    %% Plot p vector
    fprintf(' \nParameters best solution: \n')
    fprintf('\n                   R    Theta     X      L0   ')
    for i = 1:numbsh
        fprintf('\n P_elevation %g = ', i); fprintf('%.3f  ', p( Exo.nParamsSh*(i-1)+2 : Exo.nParamsSh*i+1 )' ); 
    end
    for i = 1:numbsw 
        fprintf('\n P_swivel %g    = ', i); fprintf('%.3f  ', p( (Exo.nParamsSh*(i-1)+2 + indexswivel) : (Exo.nParamsSh*i+1 + indexswivel))' ); 
    end
    fprintf('\n                   L0_wr  Perc_wr  ')
    fprintf('\n P_elbow       = '); fprintf('%.3f   ', p( end-1:end)' ); 

    %% Now analize the force of the springs
    [Pos_tot, Bod_tot, Exo_tot] = SetUp_Analysis();
    [ TAUsDesired_tot, MaxTorques_tot, robot_tot, q_tot ] = weightEffect3D( Bod_tot, Pos_tot, Exo_tot);
    [TauExo_tot, Actual_Pin, L_springs, L0, Tension, func_tension] = exoNetTorques3D(Pos_tot, Bod_tot, Exo_tot, p);
    
    %% ========================================================================
    %  GRAFICO 1: Force vs Absolute Length
    %  ========================================================================
    figure('Name','Force vs Absolute Length - All Springs (Elevation = fuchsia, Swivel = black)');
    hold on; grid on; box on;
    xlabel('Length  L (m)');
    ylabel('Force  T (N)');
    title('Force vs Absolute Length — Elevation (fuchsia) vs Swivel (black)');

    addedTheoryElevLegend  = false;
    addedSamplesElevLegend = false;
    addedTheorySwivLegend  = false;
    addedSamplesSwivLegend = false;

    % Trova la lunghezza massima per definire il range del plot
    L_max = 0;

    % Controlla lunghezze massime elevation
    if isfield(Tension,'T_dist_elevation') && ~isempty(Tension.T_dist_elevation)
        L_max = max(L_max, max(Tension.T_dist_elevation(:), [], 'omitnan'));
    end

    % Controlla lunghezze massime swivel
    if isfield(Tension,'T_dist_swivel') && ~isempty(Tension.T_dist_swivel)
        L_max = max(L_max, max(Tension.T_dist_swivel(:), [], 'omitnan'));
    end

    % Se non ci sono dati, usa un default
    if L_max == 0 || isnan(L_max)
        L_max = 2.0;
    end

    % Vettore di lunghezze assolute per le curve teoriche
    L_vec = linspace(0, L_max * 1.1, 600);

    % ============================ ELEVATION (fucsia) ============================
    if isfield(L_springs,'elevation') && ~isempty(L_springs.elevation)
        for i = 1:numbsh
            % --- L0 per questa molla ---
            if isfield(L0,'elevation') && numel(L0.elevation) >= i && ~isnan(L0.elevation(i))
                L0_i = L0.elevation(i);
            else
                
                L0_i = 1; warning('Missing L0.elevation(%d); using L0=1', i);
            end

            % --- funzione T(r) teorica ---
            ft = [];
            if isstruct(func_tension) && isfield(func_tension,'elevation') ...
               && iscell(func_tension.elevation) && numel(func_tension.elevation) >= i ...
               && isa(func_tension.elevation{i},'function_handle')
                ft = func_tension.elevation{i};
            elseif isa(func_tension,'function_handle')
                ft = func_tension;
            end

            % --- curva teorica T(L) ---
            if ~isempty(ft) && L0_i > 0
                % Converti L in r = L/L0, poi calcola T(r)
                r_vec = L_vec / L0_i;
                T_theo = arrayfun(@(x) ft(x), r_vec);
                % Plotta solo per L >= L0 (r >= 1)
                valid_idx = L_vec >= L0_i;
                if ~addedTheoryElevLegend
                    plot(L_vec(valid_idx), T_theo(valid_idx), '-', 'Color', [1 0 1 0.4], 'LineWidth', 1.2, 'DisplayName','Elevation (theory)');
                    addedTheoryElevLegend = true;
                else
                    plot(L_vec(valid_idx), T_theo(valid_idx), '-', 'Color', [1 0 1 0.3], 'LineWidth', 0.8, 'HandleVisibility','off');
                end
            end

            % --- campioni operativi (L vs T) ---
            if isfield(Tension,'T_dist_elevation') && size(Tension.T_dist_elevation,2) >= i ...
               && isfield(Tension,'elevation') && size(Tension.elevation,2) >= i
                L_samp = Tension.T_dist_elevation(:,i);
                T_i    = Tension.elevation(:,i);
                valid  = ~isnan(L_samp) & ~isnan(T_i);
                if any(valid)
                    if ~addedSamplesElevLegend
                        scatter(L_samp(valid), T_i(valid), 14, [1 0 1], 'filled', ...
                            'MarkerFaceAlpha',0.35, 'MarkerEdgeAlpha',0.35, ...
                            'DisplayName','Elevation (samples)');

                        addedSamplesElevLegend = true;
                    else
                        scatter(L_samp(valid), T_i(valid), 14, [1 0 1], 'filled', ...
                            'MarkerFaceAlpha',0.25, 'MarkerEdgeAlpha',0.25, ...
                            'HandleVisibility','off');

                    end
                end
            end
        end
    end

    % ============================== SWIVEL (nero) ==============================
    if isfield(L_springs,'swivel') && ~isempty(L_springs.swivel) && numbsw > 0
        for i = 1:numbsw
            % --- L0 per questa molla ---
            if isfield(L0,'swivel') && numel(L0.swivel) >= i && ~isnan(L0.swivel(i))
                L0_i = L0.swivel(i);
            else
                L0_i = 1; warning('Missing L0.swivel(%d); using L0=1', i);
            end

            % --- funzione T(r) teorica ---
            ft = [];
            if isstruct(func_tension) && isfield(func_tension,'swivel') ...
               && iscell(func_tension.swivel) && numel(func_tension.swivel) >= i ...
               && isa(func_tension.swivel{i},'function_handle')
                ft = func_tension.swivel{i};
            elseif isa(func_tension,'function_handle')
                ft = func_tension;
            end

            % --- curva teorica T(L) ---
            if ~isempty(ft) && L0_i > 0
                % Converti L in r = L/L0, poi calcola T(r)
                r_vec = L_vec / L0_i;
                T_theo = arrayfun(@(x) ft(x), r_vec);
                % Plotta solo per L >= L0 (r >= 1)
                valid_idx = L_vec >= L0_i;
                if ~addedTheorySwivLegend
                    plot(L_vec(valid_idx), T_theo(valid_idx), '-', 'Color', [0 0 0 0.4], 'LineWidth', 1.2, 'DisplayName','Swivel (theory)');
                    addedTheorySwivLegend = false;
                else
                    plot(L_vec(valid_idx), T_theo(valid_idx), '-', 'Color', [0 0 0 0.3], 'LineWidth', 0.8, 'HandleVisibility','off');
                end
            end

            % --- campioni operativi (L vs T) ---
            if isfield(Tension,'T_dist_swivel') && size(Tension.T_dist_swivel,2) >= i ...
               && isfield(Tension,'swivel') && size(Tension.swivel,2) >= i
                L_samp = Tension.T_dist_swivel(:,i);
                T_i    = Tension.swivel(:,i);
                valid  = ~isnan(L_samp) & ~isnan(T_i);
                if any(valid)
                    if ~addedSamplesSwivLegend
                        scatter(L_samp(valid), T_i(valid), 14, [0 0 0], 'filled', ...
                            'MarkerFaceAlpha',0.35, 'MarkerEdgeAlpha',0.35, ...
                            'DisplayName','Swivel (samples)');
                        addedSamplesSwivLegend = false;

                    else
                        scatter(L_samp(valid), T_i(valid), 14, [0 0 0], 'filled', ...
                            'MarkerFaceAlpha',0.25, 'MarkerEdgeAlpha',0.25, ...
                            'HandleVisibility','off');

                    end
                end
            end
        end
    end

    %legend('Location','best');
    
    %% ========================================================================
    %  GRAFICO 2 & 3: Force Fields con showVectField
    %  ========================================================================
    if isfield(TauExo_tot, 'elevationSh') && isfield(TauExo_tot, 'elevationEl') && ...
       isfield(TAUsDesired_tot, 'TauSh_tot') && isfield(TAUsDesired_tot, 'TauEl_tot') && ...
       isfield(Pos_tot, 'wrSwivel')
        
        % Estrai posizioni del polso (alternate)
        n_points = size(Pos_tot.wrSwivel, 1);
        idx_plane = 1:2:n_points;      % indici dispari - braccio nel piano
        idx_abducted = 2:2:n_points;   % indici pari - braccio abdotto
        
        aScale = 0.25;  % Scala per i vettori
        
        %% --- GRAFICO 2: Arm in Plane (odd indices) ---
        if ~isempty(idx_plane)
            % Crea strutture Pos filtrate per questa configurazione
            Pos_plane = Pos_tot;
            Pos_plane.wrSwivel = Pos_tot.wrSwivel(idx_plane, :);
            Pos_plane.elbowSwivel = Pos_tot.elbowSwivel(idx_plane, :);
            if size(Pos_tot.sh, 1) > 1
                Pos_plane.sh = Pos_tot.sh(idx_plane, :);
            end
            
            % Calcola i vettori di forza con indexplane=1 e Stride=1 (già filtrati)
            F_exo_plane = plotVectField3D_opt(q_tot(idx_plane,:), Bod_tot, Pos_plane, Exo_tot, robot_tot, ...
                                        TauExo_tot.elevationSh(idx_plane,:), TauExo_tot.elevationEl(idx_plane,:), ...
                                        [0, 0, 1], 1.5, 0.7, 1, 'NoPlot', true, 'Stride', 1);
            
            F_des_plane = plotVectField3D_opt(q_tot(idx_plane,:), Bod_tot, Pos_plane, Exo_tot, robot_tot, ...
                                        TAUsDesired_tot.TauSh_tot(idx_plane,:), TAUsDesired_tot.TauEl_tot(idx_plane,:), ...
                                        [1, 0, 0], 1.5, 0.7, 1, 'IsDesired', true, 'NoPlot', true, 'Stride', 1);
            
            % Estrai posizioni (già filtrate in Pos_plane)
            p0_plane = Pos_plane.wrSwivel;
            
            % Normalizza usando il massimo tra Exo e Desired
            mag_exo = max(vecnorm(F_exo_plane, 2, 2));
            mag_des = max(vecnorm(F_des_plane, 2, 2));
            mag_max_plane = max(mag_exo, mag_des);
            
            if mag_max_plane > 0
                F_exo_plane_norm = F_exo_plane / mag_max_plane;
                F_des_plane_norm = F_des_plane / mag_max_plane;
            else
                F_exo_plane_norm = F_exo_plane;
                F_des_plane_norm = F_des_plane;
            end
            
            % Plotta con showVectField
            figure('Name','Force Field - Arm in Plane (odd indices)'); 
            nancy_body;
            hold on; 
            title('Arm in Plane Configuration');
            
            hExo = showVectField(p0_plane, F_exo_plane_norm, aScale, 'b');
            hDes = showVectField(p0_plane, F_des_plane_norm, aScale, 'r');
            
            axis equal; zlim([0.6 1.9]); axis off;
            set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
            %legend([hExo, hDes], {'Exo','Desired'}, 'Location','best');
            hold off;
        end
        
        %% --- GRAFICO 3: Arm Abducted (even indices) ---
        if ~isempty(idx_abducted)
            % Crea strutture Pos filtrate per questa configurazione
            Pos_abd = Pos_tot;
            Pos_abd.wrSwivel = Pos_tot.wrSwivel(idx_abducted, :);
            Pos_abd.elbowSwivel = Pos_tot.elbowSwivel(idx_abducted, :);
            if size(Pos_tot.sh, 1) > 1
                Pos_abd.sh = Pos_tot.sh(idx_abducted, :);
            end
            
            % Calcola i vettori di forza con indexplane=1 e Stride=1 (già filtrati)
            F_exo_abd = plotVectField3D_opt(q_tot(idx_abducted,:), Bod_tot, Pos_abd, Exo_tot, robot_tot, ...
                                        TauExo_tot.elevationSh(idx_abducted,:), TauExo_tot.elevationEl(idx_abducted,:), ...
                                        [0, 0, 1], 1.5, 0.7, 1, 'NoPlot', true, 'Stride', 1);
            
            F_des_abd = plotVectField3D_opt(q_tot(idx_abducted,:), Bod_tot, Pos_abd, Exo_tot, robot_tot, ...
                                        TAUsDesired_tot.TauSh_tot(idx_abducted,:), TAUsDesired_tot.TauEl_tot(idx_abducted,:), ...
                                        [1, 0, 0], 1.5, 0.7, 1, 'IsDesired', true, 'NoPlot', true, 'Stride', 1);
            
            % Estrai posizioni (già filtrate in Pos_abd)
            p0_abd = Pos_abd.wrSwivel;
            
            % Normalizza usando il massimo tra Exo e Desired
            mag_exo = max(vecnorm(F_exo_abd, 2, 2));
            mag_des = max(vecnorm(F_des_abd, 2, 2));
            mag_max_abd = max(mag_exo, mag_des);
            
            if mag_max_abd > 0
                F_exo_abd_norm = F_exo_abd / mag_max_abd;
                F_des_abd_norm = F_des_abd / mag_max_abd;
            else
                F_exo_abd_norm = F_exo_abd;
                F_des_abd_norm = F_des_abd;
            end
            
            % Plotta con showVectField
            figure('Name','Force Field - Arm Abducted (even indices)'); 
            nancy_body_flex; 
            hold on; 
            title('Arm Abducted Configuration');
            
            hExo = showVectField(p0_abd, F_exo_abd_norm, aScale, 'b');
            hDes = showVectField(p0_abd, F_des_abd_norm, aScale, 'r');
            
            axis equal; zlim([0.6 1.9]); axis off;
            set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
            %legend([hExo, hDes], {'Exo','Desired'}, 'Location','best');
            hold off;
        end
    end

%% ========================================================================
%  GRAFICO AGGIUNTIVO 1a: Posizioni del Polso con Nancy Body
%  ========================================================================
if isfield(Pos_tot, 'wrSwivel')
    % Estrai tutte le posizioni del polso
    p_all = Pos_tot.wrSwivel;
    
    figure('Name','Wrist Positions - Nancy Body');
    nancy_body; 
    hold on;
    
    % Plotta tutte le posizioni dello stesso colore (blu)
    scatter3(p_all(:,1), p_all(:,2), p_all(:,3), 50, 'b', 'filled', ...
        'MarkerFaceAlpha', 0.6, 'DisplayName', 'Wrist Positions');
    aggiungi_piani_rotazione([0.17223, -0.0639, 1.36489], 0.85, 1, 4, [0.95 0.95 0.95], 0.13);

    title('Wrist Positions - Arm in Plane View');
    %legend('Location', 'best');
    axis equal; zlim([0.6 1.9]); axis off;
    set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
    hold off;
end

%% ========================================================================
%  GRAFICO AGGIUNTIVO 1b: Posizioni del Polso con Nancy Body Flexed
%  ========================================================================
if isfield(Pos_tot, 'wrSwivel')
    % Estrai tutte le posizioni del polso
    p_all = Pos_tot.wrSwivel;
    
    figure('Name','Wrist Positions - Nancy Body Flexed');
    nancy_body_flex;
    hold on;
    
    % Plotta tutte le posizioni dello stesso colore (blu)
    scatter3(p_all(:,1), p_all(:,2), p_all(:,3), 50, 'b', 'filled', ...
        'MarkerFaceAlpha', 0.6, 'DisplayName', 'Wrist Positions');
    aggiungi_piani_rotazione([0.17223, -0.0639, 1.36489], 0.85, 1, 4, [0.95 0.95 0.95], 0.13);

    title('Wrist Positions - Arm Abducted View');
    %legend('Location', 'best');
    axis equal; zlim([0.6 1.9]); axis off;
    set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
    hold off;
end

%% ========================================================================
%  GRAFICO AGGIUNTIVO 2: Solo Vettori Desired con Nancy Body
%  ========================================================================
if isfield(TauExo_tot, 'elevationSh') && isfield(TauExo_tot, 'elevationEl') && ...
   isfield(TAUsDesired_tot, 'TauSh_tot') && isfield(TAUsDesired_tot, 'TauEl_tot') && ...
   isfield(Pos_tot, 'wrSwivel')
    
    % Estrai posizioni del polso (alternate)
    n_points = size(Pos_tot.wrSwivel, 1);
    idx_plane = 1:2:n_points;      % indici dispari - braccio nel piano
    idx_abducted = 2:2:n_points;   % indici pari - braccio abdotto
    
    aScale = 0.25;  % Scala per i vettori
    
    %% --- Arm in Plane (odd indices) - Solo Desired ---
    if ~isempty(idx_plane)
        % Crea strutture Pos filtrate
        Pos_plane = Pos_tot;
        Pos_plane.wrSwivel = Pos_tot.wrSwivel(idx_plane, :);
        Pos_plane.elbowSwivel = Pos_tot.elbowSwivel(idx_plane, :);
        if size(Pos_tot.sh, 1) > 1
            Pos_plane.sh = Pos_tot.sh(idx_plane, :);
        end
        
        % Calcola solo i vettori desired
        F_des_plane = plotVectField3D_opt(q_tot(idx_plane,:), Bod_tot, Pos_plane, Exo_tot, robot_tot, ...
                                    TAUsDesired_tot.TauSh_tot(idx_plane,:), TAUsDesired_tot.TauEl_tot(idx_plane,:), ...
                                    [1, 0, 0], 1.5, 0.7, 1, 'IsDesired', true, 'NoPlot', true, 'Stride', 1);
        
        p0_plane = Pos_plane.wrSwivel;
        
        % Normalizza
        mag_des = max(vecnorm(F_des_plane, 2, 2));
        if mag_des > 0
            F_des_plane_norm = F_des_plane / mag_des;
        else
            F_des_plane_norm = F_des_plane;
        end
        
        % Plotta
        figure('Name','Desired Forces - Arm in Plane'); 
        nancy_body; 
        hold on; 
        title('Desired Force Field - Arm in Plane Configuration');
        
        hDes = showVectField(p0_plane, F_des_plane_norm, aScale, 'r');
        
        axis equal; zlim([0.6 1.9]); axis off;
        set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
        %legend(hDes, 'Desired', 'Location','best');
        hold off;
    end
    
    %% --- Arm Abducted (even indices) - Solo Desired ---
    if ~isempty(idx_abducted)
        % Crea strutture Pos filtrate
        Pos_abd = Pos_tot;
        Pos_abd.wrSwivel = Pos_tot.wrSwivel(idx_abducted, :);
        Pos_abd.elbowSwivel = Pos_tot.elbowSwivel(idx_abducted, :);
        if size(Pos_tot.sh, 1) > 1
            Pos_abd.sh = Pos_tot.sh(idx_abducted, :);
        end
        
        % Calcola solo i vettori desired
        F_des_abd = plotVectField3D_opt(q_tot(idx_abducted,:), Bod_tot, Pos_abd, Exo_tot, robot_tot, ...
                                    TAUsDesired_tot.TauSh_tot(idx_abducted,:), TAUsDesired_tot.TauEl_tot(idx_abducted,:), ...
                                    [1, 0, 0], 1.5, 0.7, 1, 'IsDesired', true, 'NoPlot', true, 'Stride', 1);
        
        p0_abd = Pos_abd.wrSwivel;
        
        % Normalizza
        mag_des = max(vecnorm(F_des_abd, 2, 2));
        if mag_des > 0
            F_des_abd_norm = F_des_abd / mag_des;
        else
            F_des_abd_norm = F_des_abd;
        end
        
        % Plotta
        figure('Name','Desired Forces - Arm Abducted'); 
        nancy_body_flex;
        hold on; 
        title('Desired Force Field - Arm Abducted Configuration');
        
        hDes = showVectField(p0_abd, F_des_abd_norm, aScale, 'r');
        
        axis equal; zlim([0.6 1.9]); axis off;
        set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
        %legend(hDes, 'Desired', 'Location','best');
        hold off;
    end
end

    figure('Name','Show exo'); hold on;
    showpins(Actual_Pin, Exo_tot, Pos_tot, p); zlim([0.9 1.75])

    
end
    %% Show the exo with pins

    %% Plot the torque for the shoulder for first plane of elevation (0°)
    % [Tau_desired_sh_firstplane, desidx_1] = sort(vecnorm( Torque_desired.TauSh_tot(1:2:end,:), 2, 2 ));
    %                 Tau_exo_sh_firstplane = vecnorm( Torque_Exo.elevationSh(1:2:end,:), 2, 2 );
    % 
    %     Tau_exo_sh_firstplane = Tau_exo_sh_firstplane(desidx_1);
    % 
    % timetorque = 0:length(Tau_desired_sh_firstplane)-1;  % tempo discreto, stesso numero di campioni
    % 
    % figure('Name', 'Torque Comparison 0° internal rotation / 0° elbow flexion');
    % scatter(timetorque, vecnorm(Tau_exo_sh_firstplane, 2, 2),     'b'); hold on;
    % scatter(timetorque, vecnorm(Tau_desired_sh_firstplane, 2, 2), 'r', 'filled');
    % xlabel('Frames'); ylabel('Torque (Nm)'); legend('Exoskeleton Torque', 'Desired Shoulder Torqsue'); grid on;
    % 
    % %% Plot the torque for the shoulder for second plane of elevation (90°)
    % [Tau_desired_sh_secondplane, desidx_2] = sort(vecnorm( Torque_desired.TauSh_tot(2:2:end,:), 2, 2 ));
    %                 Tau_exo_sh_secondplane = vecnorm(  Torque_Exo.elevationSh(2:2:end,:), 2, 2 );
    % 
    %     Tau_exo_sh_secondplane = Tau_exo_sh_secondplane(desidx_2);
    % 
    % timetorque = 0:length(Tau_desired_sh_secondplane)-1;  % tempo discreto, stesso numero di campioni
    % 
    % figure('Name', 'Torque Comparison 90° internal rotation / 90° elbow flexion');
    % scatter(timetorque, vecnorm(Tau_exo_sh_secondplane, 2, 2),     'b'); hold on;
    % scatter(timetorque, vecnorm(Tau_desired_sh_secondplane, 2, 2), 'r', 'filled');
    % xlabel('Frames'); ylabel('Torque (Nm)'); legend('Exoskeleton Torque', 'Desired Shoulder Torque'); grid on;

