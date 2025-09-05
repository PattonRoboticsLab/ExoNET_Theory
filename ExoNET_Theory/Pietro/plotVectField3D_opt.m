function Force_vector = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TauSh, TauEl, ...
                                                                Colr, LineWidth, MaxHeadSize, indexplane, varargin)

    scaleF = 0.2; % Graphical scale factor for force vectors
    azimuth = -150; elevation = 30; % View angles
    
    % Parse additional parameters for overlapping vectors
    p = inputParser;
    addParameter(p, 'IsDesired', false, @islogical);
    addParameter(p, 'Transparency', 1.0, @(x) x >= 0 && x <= 1);
    addParameter(p, 'Offset', [0, 0, 0], @(x) isnumeric(x) && length(x) == 3);
    parse(p, varargin{:});
    
    is_desired = p.Results.IsDesired;
    transparency = p.Results.Transparency;
    offset = p.Results.Offset;
    
    % Handle indexplane parameter
    if nargin < 10 || isempty(indexplane)
        indexplane = 1;
    end
    
    startidx = indexplane;
    
    % Select data every 2 steps starting from indexplane
    TauSh = TauSh(startidx:2:end,:); % [N x 3] - shoulder torques (x,y,z)
    TauEl = TauEl(startidx:2:end,:); % [N x 3] - elbow torques (x,y,z)
    
    % Get positions from Pos structure to build the jacobian
    if isfield(Pos, 'wrSwivel')
        wrist = Pos.wrSwivel(startidx:2:end,:); % [N x 3] - wrist positions
        elbow = Pos.elbowSwivel(startidx:2:end,:); % [N x 3] - elbow positions
        q = q(startidx:2:end,:);
        
        % Handle shoulder position (might be single position or array)
        if size(Pos.sh, 1) == 1
            % Single shoulder position, repeat for all configurations
            shoulder = repmat(Pos.sh, size(wrist, 1), 1);
        end
    else
        error('Required fields not found in Pos structure');
    end
    
    % Verify dimensions to control the size and the code is working
    if size(TauSh,2) ~= 3
        error('TauSh should have 3 columns (torque x,y,z), but has %d', size(TauSh,2));
    end
    
    if size(TauEl,2) ~= 3
        error('TauEl should have 3 columns (torque x,y,z), but has %d', size(TauEl,2));
    end
    
    % Initialize force vector array
    Force_vector = zeros(size(TauSh,1), 3);
    
    fprintf('\nPlotting image... ');
    
    % Loop on each configuration to create the Jacobian and evaluate the
    % force
    for i = 1:size(q, 1)       
        % Build the Jacobian using current arm configuration
        J = geometricJacobian(robot, q(i,:), 'forearm');
        % J = computeSimpleJacobian(p_shoulder, p_elbow, p_wrist, Bod, robot, Exo, q);
        
        % Combine shoulder and elbow torques
        % Shoulder torques affect first 3 joints
        % Elbow torque: project TauEl vector along J(:,4) direction
          v_upper = elbow(i,:) - shoulder(i,:);    % Upper arm vector
        v_forearm = wrist(i,:) - elbow(i,:);     % Forearm vector
        elbow_axis = cross(v_upper/norm(v_upper), v_forearm/norm(v_forearm));

        if norm(elbow_axis) < 0.01  % Nearly aligned segments
            % Create artificial elbow axis
            elbow_axis = cross(v_upper_norm, [0; 0; 1]);
            if norm(elbow_axis) < 0.01
                elbow_axis = cross(v_upper_norm, [0; 1; 0]);
            end
        end
    
        elbow_axis = elbow_axis / norm(elbow_axis);
        elbow_axis_normalized = elbow_axis / norm(elbow_axis); % Normalize
        
        % Project TauEl(i,:) onto the elbow axis
        tau_elbow_projected = dot( TauEl(i,:), elbow_axis_normalized );
        
        tau_total = [TauSh(i,1); TauSh(i,2); TauSh(i,3); tau_elbow_projected];
        
        % Calculate force using F = J^(-T) * tau
        % Use pseudo-inverse for robustness
        J_t = J(1:3,:)';
        f_ext = pinv(J_t) * tau_total; % [3x1] force vector
        Force_vector(i,:) = f_ext(1:3)';
       
    end % close for cycle

    % Remove configurations with zero forces for statistics
    valid_forces = Force_vector(vecnorm(Force_vector, 2, 2) > 1e-10, :);
    
    if isempty(valid_forces) % handle case where forces are zeros
        warning('All force vectors are zero or invalid');
        fprintf('Check input torques and arm configuration\n');
        return;
    end
    
    % Handle singularities and outliers for better visualization
    force_magnitudes = vecnorm(valid_forces, 2, 2);
    
    % Method 1: Use percentile-based normalization to ignore extreme outliers
    F_percentile = prctile(force_magnitudes, 95); % Use 95th percentile
    
    % Method 2: Use median + k*MAD (Median Absolute Deviation) for robust scaling
    median_force = median(force_magnitudes);
    mad_force = mad(force_magnitudes, 1); % MAD with median
    F_robust = median_force + 3 * mad_force; % 3-sigma equivalent
    
    % Method 3: Clamp extreme values
    F_clamp_threshold = median_force + 5 * mad_force;
    
    % Choose normalization method, set to percentile initially
    normalization_method = 'percentile'; % Options: 'max', 'percentile', 'robust', 'clamp'
    
    switch normalization_method
        case 'max'
            F_max = max(force_magnitudes);
        
        case 'percentile'
            F_max = F_percentile;
        
        case 'robust'
            F_max = F_robust;
        
        case 'clamp'
            F_max = F_clamp_threshold;
            % Clamp the force vectors
            for i = 1:size(Force_vector,1)
                force_mag = vecnorm(Force_vector(i,:));
                if force_mag > F_clamp_threshold
                    Force_vector(i,:) = Force_vector(i,:) / force_mag * F_clamp_threshold;
                end
            end
    end
    
    % Create figure if not already open
    if ~ishold
        figure;
    end
    hold on;
    
    %% ===================== Plot each force vector========================
    plotted_vectors = 0;    extreme_vectors = 0;
    
    for i = 1:size(Force_vector,1)
        force_magnitude = vecnorm(Force_vector(i,:));
        if force_magnitude > 1e-10 % Only plot non-zero vectors
            
            % Check if this is an extreme vector (magnitude is very high)
            is_extreme = false;
            if strcmp(normalization_method, 'percentile') && force_magnitude > F_percentile * 2
                is_extreme = true;
                extreme_vectors = extreme_vectors + 1;
            elseif strcmp(normalization_method, 'robust') && force_magnitude > F_robust * 2
                is_extreme = true;
                extreme_vectors = extreme_vectors + 1;
            end
            
            % Scale force vector
             F = Force_vector(i,:) / F_max * scaleF;
            p0 = wrist(i,:) + offset; % Apply offset for overlapping vectors
            
            % Determine color and transparency for the legends
            if is_desired
                % Red for desired vectors
                color = [1, 0, 0]; % Red
                alpha = transparency;
                marker_color = 'r';
                legend_label = 'Desired Forces';
            else
                % Blue for actual vectors
                color = [0, 0, 1]; % Blue
                alpha = transparency;
                marker_color = 'b';
                legend_label = 'Actual Forces';
            end
            
            % Modify appearance for extreme vectors (from singularities)
            if is_extreme
                % Make extreme vectors more transparent and thinner
                alpha = alpha * 0.5;
                current_linewidth = LineWidth * 0.5;
                marker_size = 4; % Smaller marker
            else
                current_linewidth = LineWidth;
                marker_size = 8;
            end
            
            % Plot dot for wrist position with transparency
            scatter3(p0(1), p0(2), p0(3), marker_size, marker_color, 'filled', ...
                'MarkerFaceAlpha', alpha, 'MarkerEdgeAlpha', alpha);
            
            % Plot force vector as quiver with transparency
            h = quiver3(p0(1), p0(2), p0(3), ...
                F(1), F(2), F(3), ...
                'Color', color, 'LineWidth', current_linewidth, 'MaxHeadSize', MaxHeadSize);
            
            % Set transparency for the quiver
            h.Color(4) = alpha; % Set alpha channel
                       
            plotted_vectors = plotted_vectors + 1;
        end
    end
    
    % Report plotting statistics
    if extreme_vectors > 0
        fprintf('Plotted %d vectors (%d extreme from singularities)\n', plotted_vectors, extreme_vectors);
    else
        fprintf('Plotted %d vectors\n', plotted_vectors);
    end
    
    % Set plot properties
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); grid on;
    axis equal; % Keep proportions
    view(3); view(azimuth, elevation); 
    
    % Add title and legend
    if is_desired
        title(sprintf('Desired Force Vector Field (4-DOF Arm)\n'));
    else
        title(sprintf('Actual Force Vector Field (4-DOF Arm)\n' ));
    end
       
    drawnow;    
    
end

% Example usage for plotting both vectors with legend:
%
% figure;
% % Plot actual forces (blue)
% [Force_actual, h1] = plotVectField3D(q, Bod, Pos, Exo, robot, TauSh_actual, TauEl_actual, ...
%     [0, 0, 1], LineWidth, MaxHeadSize, indexplane, 'Transparency', 0.7);
%
% % Plot desired forces (red) with slight offset
% [Force_desired, h2] = plotVectField3D(q, Bod, Pos, Exo, robot, TauSh_desired, TauEl_desired, ...
%     [1, 0, 0], LineWidth, MaxHeadSize, indexplane, 'IsDesired', true, ...
%     'Transparency', 0.9, 'Offset', [0.001, 0.001, 0.001]);
%
% % Add legend using the returned handles
% legend([h1(1), h2(1)], 'Actual Forces', 'Desired Forces', 'Location', 'best');
%
% % Alternative: Automatic legend (if DisplayName is set)
% legend('show', 'Location', 'best');