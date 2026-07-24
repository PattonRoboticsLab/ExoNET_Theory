function out = compareVectors(Force_vector, varargin)
% compareVectors  Compare desired vs exo (extended & flexed) in 3D.
% - Sums vectors component-wise (X,Y,Z) for the fields:
%     desired_extended, exo_extended, desired_flexed, exo_flexed
% - Plots two 3D subplots (left = extended, right = flexed)
% - Draws the angle arc between the two resultant vectors with θ text
% - Shows a summary BOX **below** each subplot (annotation), with:
%     |D|, |E|, Δ|v|, Δ|v|% vs desired, θ (deg), direction (same/opposite), alignment%
%
% Usage:
%   out = compareVectors(Force_vector, 'Scale',1.0, 'Origin',[0 0 0], ...
%                        'OriginExtended',[...], 'OriginFlexed',[...], ...
%                        'Normalize',false, 'Figure',[])
%
% Options:
%   'Scale'           (default 1.0)  visual scale factor for arrows
%   'Normalize'       (default false) if true, arrows are normalized (graphics only)
%   'Origin'          (default [0 0 0]) origin for both subplots
%   'OriginExtended'  (override origin for the extended subplot)
%   'OriginFlexed'    (override origin for the flexed subplot)
%   'Figure'          figure handle to reuse (if empty, creates a new figure)

% ===== Options =====
p = inputParser;
addParameter(p,'Scale',1.0);
addParameter(p,'Normalize', true);
addParameter(p,'Origin',[0 0 0]);
addParameter(p,'OriginExtended',[]);
addParameter(p,'OriginFlexed',[]);
addParameter(p,'Figure',[]);
parse(p,varargin{:});
opt = p.Results;

if isempty(opt.OriginExtended), opt.OriginExtended = opt.Origin; end
if isempty(opt.OriginFlexed),   opt.OriginFlexed   = opt.Origin; end

% Colors
C.exo     = [0 0 1];  % blue
C.desired = [1 0 0];  % red
C.delta   = [0 1 0];  % green

% Fetch and component-wise sum 
desExt = Force_vector.desired_extended;
exoExt = Force_vector.exo_extended;
desFlx = Force_vector.desired_flexed;
exoFlx = Force_vector.exo_flexed;  % FIXED: removed space before dot

out.extended.desired = sum(desExt,1);   % 1x3
out.flexed.desired   = sum(desFlx,1);   % 1x3

out.extended.exo     = sum(exoExt,1);   % 1x3
out.flexed.exo       = sum(exoFlx,1);   % 1x3

% ====== Normalization ======
V = [
    out.extended.desired;   % 1
    out.flexed.desired;     % 2
    out.extended.exo;       % 3
    out.flexed.exo          % 4
];

mags = vecnorm(V,2,2);     

[magMax, idxMax] = max(mags);    vecMax  = V(idxMax,:);    

% ===== Metrics =====
out.extended.metrics = makeMetrics(out.extended.desired, out.extended.exo);
out.flexed.metrics   = makeMetrics(out.flexed.desired,  out.flexed.exo);

% ===== Figure =====
if isempty(opt.Figure) || ~ishandle(opt.Figure)
    fig = figure('Name','Desired vs Exo (Extended & Flexed)');
else
    fig = figure(opt.Figure); clf(fig);
end
set(fig,'Units','normalized');  % for consistent annotation placement

% ---------- EXTENDED ----------
ax1 = subplot(1,2,1,'Parent',fig); 
try
    nancy_body;
    fprintf('nancy_body() loaded successfully for Extended\n');
catch ME
    fprintf('Warning: nancy_body() failed: %s\n', ME.message);
end
hold(ax1,'on'); axis(ax1,'equal'); view(ax1,3);
xlabel(ax1,'Frontal plane'); ylabel(ax1,'Sagital plane'); zlabel(ax1,'Trasverse plane');
title(ax1,'Extended');
drawPanel(ax1, opt.OriginExtended, out.extended.desired, out.extended.exo, opt, C, out.extended.metrics, magMax);

% ---------- FLEXED ----------
ax2 = subplot(1,2,2,'Parent',fig); 
try
    nancy_body_flex;
    fprintf('nancy_body_flex() loaded successfully for Flexed\n');
catch ME
    fprintf('Warning: nancy_body_flex() failed: %s\n', ME.message);
    % Try alternative names
    alt_names = {'nancy_body_flexed', 'nancybody_flex', 'nancy_flex', 'nancy_body_flesso'};
    success = false;
    for i = 1:length(alt_names)
        try
            feval(alt_names{i});
            fprintf('%s() worked as alternative!\n', alt_names{i});
            success = true;
            break;
        catch
            % Continue trying
        end
    end
    if ~success
        fprintf('No working flexed body function found. Continuing without body...\n');
    end
end
hold(ax2,'on'); axis(ax2,'equal'); view(ax2,3);
xlabel(ax2,'Frontal plane'); ylabel(ax2,'Sagital plane'); zlabel(ax2,'Trasverse plane');
title(ax2,'Flexed');
drawPanel(ax2, opt.OriginFlexed, out.flexed.desired, out.flexed.exo, opt, C, out.flexed.metrics, magMax);

end % ===== end main function =====

% -------------------------------------------------------------------------
%                                HELPERS
% -------------------------------------------------------------------------
function [U, handles] = drawPanel(ax, o, d, e, opt, C, met, vectmax)
    % Graphics-only normalization (metrics are computed on true vectors)
    dd = d;    ee = e;
    if opt.Normalize
        if vectmax > 0
            dd = dd / vectmax;
            ee = ee / vectmax;
        end
    end

    dd = dd * opt.Scale;     ee = ee * opt.Scale;

    % arrows from origin
        hD = quiver3(ax, o(1),o(2),o(3), dd(1),dd(2),dd(3), 0, 'LineWidth',1.5, 'Color', C.desired, 'MaxHeadSize',0.7, 'AutoScale','off');
        hE = quiver3(ax, o(1),o(2),o(3), ee(1),ee(2),ee(3), 0, 'LineWidth',1.5, 'Color', C.exo,     'MaxHeadSize',0.7, 'AutoScale','off');
    hError = quiver3(ax, o(1),o(2),o(3), dd(1)-ee(1), dd(2)-ee(2), dd(3)-ee(3), 0, 'LineWidth',1, 'Color', C.delta,   'MaxHeadSize',14, 'AutoScale','off');

    % tip-to-tip difference (dashed line)
    pD = o + dd; pE = o + ee;
    % hL = plot3(ax, [pD(1) pE(1)], [pD(2) pE(2)], [pD(3) pE(3)], ...
    %     '--', 'Color', C.delta, 'LineWidth',2);

    % angle arc WITH text on the arc
    try
        rArc = 0.35 * max(norm(dd), norm(ee));
        if rArc < 0.05, rArc = 0.05; end  % minimum radius
        drawAngleArc(ax, o, d, e, rArc, sprintf('\\theta = %.1f^\\circ', met.angle_deg));
    catch ME
        fprintf('Warning: Could not draw angle arc: %s\n', ME.message);
    end

    % legend (names only)
    legend(ax, [hD hE hError], {'Desired','Exo\_opt','Error'}, 'Location','best');

    % ===== Numeric LABEL BELOW the subplot (annotation) =====
    fig  = ancestor(ax, 'figure');
    oldUnitsFig = get(fig,'Units');    set(fig,'Units','normalized');
    oldUnitsAx  = get(ax,'Units');     set(ax,'Units','normalized');
    pos  = get(ax,'Position');         % [left bottom width height], normalized
    
    % DEBUG: Print position info
    fprintf('Subplot position: [%.3f %.3f %.3f %.3f]\n', pos(1), pos(2), pos(3), pos(4));
    
    txt  = sprintf(['|D|=%.3g   |E|=%.3g   ' ...
                        '\\Delta|v|=%.3g (%.1f%%%% vs D)   ' ...
                        '\\theta=%.2f^\\circ   direction=%s   '], ...
                        met.mag_desired, met.mag_exo, ...
                        met.delta_mag, met.delta_mag_pct, ...
                        met.angle_deg, met.direction);

    % area under the subplot
    hBox = 0.08;   % box height (normalized)
    gap  = 0.02;   % gap below the axes
    yPos = max(0.01, pos(2) - gap - hBox);
    
    % DEBUG: Print annotation position
    fprintf('Annotation position: [%.3f %.3f %.3f %.3f]\n', pos(1), yPos, pos(3), hBox);

    try
        h_annotation = annotation(fig, 'textbox', [pos(1), yPos, pos(3), hBox], ...
            'String', txt, 'Interpreter','tex', ...
            'HorizontalAlignment','center', 'VerticalAlignment','middle', ...
            'EdgeColor','black', 'BackgroundColor','white', 'FontSize', 9);
        fprintf('Annotation created successfully!\n');
    catch ME
        fprintf('ERROR creating annotation: %s\n', ME.message);
        % Fallback: use text in the axes
        text(ax, mean([pD(1) pE(1)]), min([pD(2) pE(2)])-0.1, min([pD(3) pE(3)]), ...
            sprintf('|D|=%.2f |E|=%.2f θ=%.1f°', met.mag_desired, met.mag_exo, met.angle_deg), ...
            'FontSize', 8, 'BackgroundColor', 'white', 'HorizontalAlignment', 'center');
    end

    % restore units
    set(ax,'Units',oldUnitsAx); set(fig,'Units',oldUnitsFig); axis equal;

    % points used for global limits
    U = [o; pD; pE];
    handles = struct('hD',hD,'hE',hE);
end

function met = makeMetrics(d, e)
    md = norm(d); 
    me = norm(e);
    met.mag_desired = md;
    met.mag_exo     = me;
    met.delta_mag   = me - md;                       % magnitude difference
    if md > 0
        met.delta_mag_pct = 100*(me - md)/md;        % FIXED: correct formula (me-md)/md
    else
        met.delta_mag_pct = NaN;
    end
    if md>0 && me>0
        c = max(-1, min(1, dot(d,e)/(md*me)));
        met.angle_deg      = acosd(c);               % direction difference (angle)
        met.direction      = ternary(c>=0, 'same', 'opposite');
    else
        met.angle_deg      = NaN;
        met.direction      = 'undefined';
    end
    met.delta_vec = e - d;                            % tip-to-tip difference
end

function drawAngleArc(ax, o, d, e, r, labelStr)
% drawAngleArc  Draw the arc of the angle between vectors d and e (3D).
% ax        target axes
% o         [1x3] origin
% d, e      [1x3] vectors (not normalized)
% r         arc radius
% labelStr  string to draw on the arc

    if r <= 0, return; end
    if norm(d) == 0 || norm(e) == 0, return; end

    % column vectors (3x1)
    u = d(:) / norm(d);
    w = e(:) / norm(e);
    n = cross(u, w);

    if norm(n) < eps
        % Almost parallel: pick any direction orthogonal to u
        B = null(u.');          % orthonormal basis of u-complement
        if isempty(B), return; end
        v = B(:,1);
        th = linspace(0, 0.1, 32);    % small arc
    else
        % Build a basis in the plane of u and w
        v = w - (u.'*w)*u; 
        v = v / norm(v);
        theta = acos(max(-1, min(1, u.'*w)));
        th = linspace(0, theta, 128);
    end

    % Arc (3xN)
    arc = o(:) + r*(u*cos(th) + v*sin(th));

    plot3(ax, arc(1,:), arc(2,:), arc(3,:), '-', 'LineWidth',2, 'Color',[0.1 0.1 0.1]);

    % Place label on the arc
    if ~isempty(labelStr) && length(th) > 1
        tc = th(round(end/2));
        pc = o(:) + 1.05*r*(u*cos(tc) + v*sin(tc));
        text(ax, pc(1), pc(2), pc(3), labelStr, ...
            'Color',[0.1 0.1 0.1], 'FontWeight','bold', 'FontSize', 10, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'BackgroundColor', [1 1 1 0.8], 'EdgeColor', [0.5 0.5 0.5]);
    end
end

function y = ternary(cond, a, b)
    if cond, y=a; else, y=b; end
end