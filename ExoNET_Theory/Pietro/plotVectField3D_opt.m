function Force_vector = plotVectField3D_opt( q, Bod, Pos, Exo, robot, TauSh, TauEl, ...
        Colr, LineWidth, MaxHeadSize, indexplane, varargin)

    %% --------- Optional Parameters ---------
    p = inputParser;
    addParameter(p, 'IsDesired',        false,     @islogical);
    addParameter(p, 'Transparency',     1.0,       @(x) isnumeric(x) && isscalar(x) && x>=0 && x<=1);
    addParameter(p, 'Offset',           [0 0 0],   @(x) isnumeric(x) && numel(x)==3);
    addParameter(p, 'Stride',           2,         @(x) isnumeric(x) && isscalar(x) && x>=1);
    addParameter(p, 'Tol',              1e-6,      @(x) isnumeric(x) && isscalar(x) && x>0);
    addParameter(p, 'BodyName',         'hand',    @(s) ischar(s) || isstring(s));
    addParameter(p, 'AssumeZeroMoment', true,      @islogical);   % true = only F (M=0 at application point)
    addParameter(p, 'ReturnWrench',     false,     @islogical);   % always returns Force_vector
    addParameter(p, 'TorquesAreJoint',  false,     @islogical);   % true = TauSh/TauEl already in joint coords
    addParameter(p, 'TauAreActuator',   false,     @islogical);   % if true: τ_actuator -> flip sign
    addParameter(p, 'ElbowAxisSign',    +1,        @(x) isnumeric(x) && isscalar(x) && (x==1 || x==-1));
    addParameter(p, 'ForceApplicationPoint', 'auto', @(s) ischar(s) || isstring(s)); % 'wrist', 'elbow', 'auto'
    addParameter(p, 'NoPlot', false, @islogical);
    parse(p, varargin{:});

    NoPlot            = p.Results.NoPlot;
    is_desired        = p.Results.IsDesired;
    transparency      = p.Results.Transparency;
    offset            = p.Results.Offset(:).';
    stride            = p.Results.Stride;
    tol               = p.Results.Tol;
    bodyName          = char(p.Results.BodyName);
    assumeZeroMoment  = p.Results.AssumeZeroMoment;
    returnWrench      = p.Results.ReturnWrench; %#ok<NASGU>
    torquesAreJoint   = p.Results.TorquesAreJoint;
    tauAreAct         = p.Results.TauAreActuator;
    elbowAxisSign     = p.Results.ElbowAxisSign;
    forceAppPoint     = char(p.Results.ForceApplicationPoint);

    %% --------- Graphics Setup / Sampling ---------
    scaleF   = 0.2;
    azimuth  = -150; elevation = 30;
    if nargin < 11 || isempty(indexplane), indexplane = 1; end
    idx = indexplane:stride:size(q,1);

    TauSh = TauSh(idx, :);   % [N x 3] Mx,My,Mz (world) or torques to project
    TauEl = TauEl(idx, :);   % [N x 3]
    q     = q(idx, :);

    % Positions (for plotting)
    if isfield(Pos,'wrSwivel') && isfield(Pos,'elbowSwivel') && isfield(Pos,'sh')
        wrist  = Pos.wrSwivel(idx, :);
        elbow  = Pos.elbowSwivel(idx, :);
        if size(Pos.sh,1)==1, shoulder = repmat(Pos.sh, numel(idx), 1);
        else,                 shoulder = Pos.sh(idx, :);
        end
    else
        error('Pos must contain: sh, elbowSwivel, wrSwivel');
    end

    %% --------- Validation ---------
    if size(TauSh,2) ~= 3 || size(TauEl,2) ~= 3
        error('TauSh/TauEl must have 3 columns (Mx,My,Mz).');
    end
    if size(q,2) ~= 4
        error('q must be [N x 4] = [shoulderZ, shoulderY, shoulderZ2, elbowY].');
    end

    N = size(q,1);
    Force_vector = zeros(N,3);

    % Determine force application point and corresponding body
    if strcmp(forceAppPoint, 'auto')
        if contains(lower(bodyName), 'forearm') || contains(lower(bodyName), 'elbow')
            forceAppPoint = 'elbow';
            plotPositions = elbow;
        else
            forceAppPoint = 'wrist';
            plotPositions = wrist;
        end
    elseif strcmp(forceAppPoint, 'elbow')
        plotPositions = elbow;
    elseif strcmp(forceAppPoint, 'wrist')
        plotPositions = wrist;
    else
        error('ForceApplicationPoint must be ''auto'', ''elbow'', or ''wrist''');
    end

    %fprintf('Computing forces at %s using body ''%s''\n', forceAppPoint, bodyName);

    %% --------- Main Loop ---------
    for i = 1:N
        qi = q(i,:);

        % --- Joint Torques (4x1) ---
        if torquesAreJoint
            % TauSh(:,1:3) = [τ_shZ, τ_shY, τ_shZ2], TauEl(:,1) = τ_elbowY
            tau_total = [TauSh(i,1); TauSh(i,2); TauSh(i,3); TauEl(i,1)];
        else
            % Project spatial moments (world) onto actual joint axes (world)
            R_shZ  = tform2rotm(getTransform(robot, qi, 'shoulder_abduction')); % Z
            R_shY  = tform2rotm(getTransform(robot, qi, 'humeral_elevation'));  % Y
            R_shZ2 = tform2rotm(getTransform(robot, qi, 'humeral_rotation'));   % Z (axial)
            R_el   = tform2rotm(getTransform(robot, qi, 'forearm'));            % elbow Y

            ax_shZ_world  = R_shZ  * [0;0;1];
            ax_shY_world  = R_shY  * [0;1;0];
            ax_shZ2_world = R_shZ2 * [0;0;1];   % Correct: Z, not X
            ax_el_world   = R_el   * [0; elbowAxisSign; 0];   % Use +1 or -1 if elbow has opposite direction

            tau_shZ  = dot(TauSh(i,:).', ax_shZ_world);
            tau_shY  = dot(TauSh(i,:).', ax_shY_world);
            tau_shZ2 = dot(TauSh(i,:).', ax_shZ2_world);
            tau_el   = dot(TauEl(i,:).',  ax_el_world);

            tau_total = [tau_shZ; tau_shY; tau_shZ2; tau_el]; % order [Z,Y,Z,ElbowY]
        end
        
        if tauAreAct
            tau_total = -tau_total;   % τ_act + J^T w_ext = 0  =>  w_ext = -pinv(J^T) τ_act
        end

        % --- CRITICAL: Adjust torques based on force application point ---
        if strcmp(forceAppPoint, 'elbow')
            % Forces at elbow: only first 3 joints contribute (shoulder joints)
            % Elbow joint doesn't contribute to forces AT the elbow
            tau_for_forces = tau_total(1:3);  % Only shoulder joints [τ_shZ, τ_shY, τ_shZ2]
            n_joints = 3;
        else % wrist
            % Forces at wrist: all 4 joints contribute
            tau_for_forces = tau_total;  % All joints [τ_shZ, τ_shY, τ_shZ2, τ_elbow]
            n_joints = 4;
        end

        % --- Jacobian at the specified body (world frame) ---
        J_full = geometricJacobian(robot, qi, bodyName);   % 6x4, [Jω; Jv]
        
        % Extract relevant columns based on force application point
        if strcmp(forceAppPoint, 'elbow')
            J = J_full(:, 1:3);  % Only first 3 joints (shoulder)
        else
            J = J_full;          % All 4 joints
        end

        % --- Force/Wrench Estimation ---
        if assumeZeroMoment
            % τ ≈ Jv^T F (F in world frame)
            JvT = J(4:6, :).';                          % [3 x n_joints]
            if rank(JvT) < 3
                warning('Jacobian is singular at pose %d. Using pseudo-inverse.', i);
            end
            F = pinv(JvT, tol) * tau_for_forces;       % [3x1] -> [Fx,Fy,Fz]
            Force_vector(i,:) = F(:).';
        else
            % J^T * wrench = τ (wrench = [Fx;Fy;Fz; Mx;My;Mz] in world)
            if rank(J) < min(size(J))
                warning('Full Jacobian is singular at pose %d. Using pseudo-inverse.', i);
            end
            wrench = pinv(J.', tol) * tau_for_forces;  % [6x1]
            Force_vector(i,:) = wrench(1:3).';
        end
    end

    %% --------- Cleanup / Scale ---------
    mags  = vecnorm(Force_vector,2,2);
    valid = mags > 1e-10 & all(isfinite(Force_vector),2);
    if ~any(valid)
        warning('All force vectors are null/invalid. Check inputs and poses.');
        return;
    end
    F_max = max(mags(valid));

%% --------- Plot ---------
if ~NoPlot
    if ~ishold, figure; end
    hold on;

    if nargin >= 8 && ~isempty(Colr), baseColor = Colr;
    else, baseColor = is_desired*[1 0 0] + (~is_desired)*[0 0 1];
    end
    alpha = transparency; marker_color = baseColor;

    for i = 1:N
        if ~valid(i), continue; end
        Fplot = Force_vector(i,:) / F_max * scaleF;
        p0 = plotPositions(i,:) + offset;

        scatter3(p0(1), p0(2), p0(3), 8, marker_color, 'filled', 'MarkerFaceAlpha', alpha, 'MarkerEdgeAlpha', alpha);

        h = quiver3(p0(1), p0(2), p0(3), Fplot(1), Fplot(2), Fplot(3), ...
                    'Color', baseColor, 'LineWidth', LineWidth, 'MaxHeadSize', MaxHeadSize, 'AutoScale','off');
        
        try, h.Color(4) = alpha; catch, end
    end

    xlabel('Frontal plane'); ylabel('Sagital plane'); zlabel('Trasversal plane'); grid off; set(gca, 'XTick', [], 'YTick', [], 'ZTick', []); axis equal;
    view(3); view(azimuth, elevation);
    title(sprintf('%s Force Vector Field at %s (4-DOF Arm)\n', ...
          tern(is_desired,'Desired','Actual'), upper(forceAppPoint)));
    drawnow;
end

end

function out = tern(cond,a,b)
    if cond, out = a; else, out = b; end
end