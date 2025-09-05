% PlotVectField3D_Simple: plot the vector field using existing kinematics
% Uses the positions already calculated in Pos structure
% Formula: F = J^(-T) * tau
%
% Inputs:
%   q: joint angles [N x 4] - [abd, elev, rot, elbow] (not used, kept for compatibility)
%   TauSh: shoulder torques [N x 3] - [tau_x, tau_y, tau_z] w.r.t. global axes
%   TauEl: elbow torques [N x 3] - [tau_x, tau_y, tau_z] w.r.t. global axes
%   Pos: structure with positions (from forwardKin3D)
%   Bod: body parameters with lengths L(1), L(2)
%   Exo: exoskeleton parameters
%   Other parameters for plotting
function Force_vector = plotVectField3D(q, Bod, Pos, Exo, robot, TauSh, TauEl, ...
    Colr, LineWidth, MaxHeadSize, indexplane)

    scaleF = 0.2; % Graphical scale factor for force vectors
    azimuth = -150; elevation = 30; % View angles
    
    % Handle indexplane parameter
    if nargin < 10 || isempty(indexplane)
        indexplane = 1;
    end
    
    startidx = indexplane;
    
    % Select data every 2 steps starting from indexplane
    TauSh = TauSh(startidx:2:end,:); % [N x 3] - shoulder torques (x,y,z)
    TauEl = TauEl(startidx:2:end,:); % [N x 3] - elbow torques (x,y,z)
    
    % Get positions from Pos structure
    if isfield(Pos, 'wrSwivel')
        wrist = Pos.wrSwivel(startidx:2:end,:); % [N x 3] - wrist positions
        elbow = Pos.elbowSwivel(startidx:2:end,:); % [N x 3] - elbow positions
        
        % Handle shoulder position (might be single position or array)
        if size(Pos.sh, 1) == 1
            % Single shoulder position, repeat for all configurations
            shoulder = repmat(Pos.sh, size(wrist, 1), 1);
        else
            shoulder = Pos.sh(startidx:2:end,:); % [N x 3] - shoulder positions
        end
    else
        error('Required fields not found in Pos structure');
    end
    
    % Verify dimensions
    if size(TauSh,2) ~= 3
        error('TauSh should have 3 columns (torque x,y,z), but has %d', size(TauSh,2));
    end
    
    if size(TauEl,2) ~= 3
        error('TauEl should have 3 columns (torque x,y,z), but has %d', size(TauEl,2));
    end
    
    % Initialize force vector array
    Force_vector = zeros(size(TauSh,1), 3);
    
    fprintf('\nPlotting image... ');
    
    % Loop on each configuration
    for i = 1:size(TauSh, 1)
        try
            % Get current positions
            p_shoulder = shoulder(i,:)';
            p_elbow = elbow(i,:)';
            p_wrist = wrist(i,:)';
            
            % Compute simple Jacobian using current arm configuration
            J = computeSimpleJacobian(p_shoulder, p_elbow, p_wrist, Bod);
            
            % Combine shoulder and elbow torques
            % Assume shoulder torques affect first 3 joints, elbow torque affects last joint
            tau_total = [TauSh(i,1); TauSh(i,2); TauSh(i,3); TauEl(i,1)]; % Take x-component of elbow torque
            
            % Calculate force using F = J^(-T) * tau
            % Use pseudo-inverse for robustness
            J_t = J';
            if rank(J_t) >= 3  % Check if we can solve for forces
                f_ext = pinv(J_t) * tau_total; % [3x1] force vector
                Force_vector(i,:) = f_ext(1:3)';
            else
                % For singular configurations, use damped least squares
                lambda = 0.01; % Damping factor
                f_ext = J_t * inv(J_t' * J_t + lambda^2 * eye(size(J_t' * J_t))) * tau_total;
                Force_vector(i,:) = f_ext(1:3)';
                % warning('Singular configuration at point %d', i);
            end
            
        catch ME
            warning('Error at configuration %d: %s', i, ME.message);
            Force_vector(i,:) = [0, 0, 0]; % Set to zero if error
        end
    end
    
    % Remove configurations with zero forces for statistics
    valid_forces = Force_vector(vecnorm(Force_vector, 2, 2) > 1e-10, :);
    
    if isempty(valid_forces)
        warning('All force vectors are zero or invalid');
        fprintf('Check input torques and arm configuration\n');
        return;
    end
    
    % Plotting
    F_max = max(vecnorm(valid_forces, 2, 2)); % Maximum norm of valid forces
    
    % Create figure if not already open
    if ~ishold
        figure;
    end
    hold on;
    
    % Plot each force vector
    plotted_vectors = 0;
    for i = 1:size(Force_vector,1)
        force_magnitude = vecnorm(Force_vector(i,:));
        if force_magnitude > 1e-10 % Only plot non-zero vectors
            % Scale force vector
            F = Force_vector(i,:) / F_max * scaleF;
            p0 = wrist(i,:);
            
            % Plot black dot for wrist position
            scatter3(p0(1), p0(2), p0(3), 8, 'k', 'filled');
            
            % Plot force vector as quiver
            quiver3(p0(1), p0(2), p0(3), ...
                F(1), F(2), F(3), ...
                'Color', Colr, 'LineWidth', LineWidth, 'MaxHeadSize', MaxHeadSize);
            
            plotted_vectors = plotted_vectors + 1;
        end
    end
    
    % Set plot properties
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 
    grid on;
    axis equal; % Keep proportions
    view(3); 
    view(azimuth, elevation); 
    
    % Add title and legend
    title(sprintf('Force Vector Field (4-DOF Arm)\nMax Force: %.3f N', F_max));
    
    % Add colorbar to show force magnitude
    if plotted_vectors > 0
        colormap(jet);
        c = colorbar;
        c.Label.String = 'Force Magnitude [N]';
        caxis([0, F_max]);
    end
    
    drawnow;    
    
end

