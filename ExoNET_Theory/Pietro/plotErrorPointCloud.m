% ===================== FUNZIONE DI SUPPORTO =====================
function plotErrorPointCloud(Pos, err_vec, varargin)
% plotErrorPointCloud(Pos, err_vec, 'which','wrist'|'elbow', 'N',Nused, 'titleStr','...')
% Colora i punti in rosso con intensità e trasparenza proporzionali all'errore.

p = inputParser;
addParameter(p,'which','wrist');
addParameter(p,'N',[]);
addParameter(p,'titleStr','Error point cloud');
addParameter(p,'alphaMax',0.9);      % trasparenza massima
addParameter(p,'mSize',20);          % grandezza marker
parse(p,varargin{:});
opt = p.Results;

% Seleziona i punti (wrist o elbow)
switch lower(opt.which)
    case 'wrist'
        P = Pos.wrSwivel;
    case 'elbow'
        P = Pos.elbowSwivel;
    otherwise
        error('which deve essere ''wrist'' o ''elbow''.');
end

% Numero di campioni da usare (se fornito)
if ~isempty(opt.N)
    P = P(1:opt.N, :);
    err_vec = err_vec(1:opt.N);
else
    N = min(size(P,1), numel(err_vec));
    P = P(1:N,:); err_vec = err_vec(1:N);
end

% Normalizzazione errore (robusta, evita outlier)
cmax = prctile(abs(err_vec), 99);
if cmax <= eps, cmax = 1; end
eNorm = min(abs(err_vec)/cmax, 1);            % [0,1]

% Colore: rosso con intensità = eNorm
C = [eNorm, zeros(size(eNorm)), zeros(size(eNorm))];

% Proviamo alpha per-punto (MATLAB moderni)
supportsPerPointAlpha = isprop(scatter3(0,0,0), 'AlphaData'); delete(findobj(gca,'Type','scatter'));
try
    s = scatter3(P(:,1), P(:,2), P(:,3), opt.mSize, C, 'filled', ...
                 'MarkerEdgeColor','none');
    axis equal; grid on; view(3);
    xlabel X; ylabel Y; zlabel Z; title(opt.titleStr);

    if supportsPerPointAlpha
        % Per-punto: usare AlphaData + 'flat'
        set(s, 'MarkerFaceAlpha','flat', 'AlphaData', eNorm*opt.alphaMax, ...
               'AlphaDataMapping','none');
    else
        % Fallback: binnig su 8 livelli di trasparenza
        delete(s);
        nb = 8;
        edges = linspace(0,1,nb+1);
        hold on;
        for b = 1:nb
            idx = eNorm >= edges(b) & eNorm < edges(b+1);
            if ~any(idx), continue; end
            alpha_b = ((edges(b)+edges(b+1))/2) * opt.alphaMax;
            c_b = [ones(sum(idx),1)*((edges(b)+edges(b+1))/2), zeros(sum(idx),1), zeros(sum(idx),1)];
            scatter3(P(idx,1), P(idx,2), P(idx,3), opt.mSize, c_b, 'filled', ...
                     'MarkerEdgeColor','none', 'MarkerFaceAlpha', alpha_b);
        end
        hold off;
    end

    % Estetica
    camlight headlight; lighting gouraud;
    box on; axis vis3d;

    % Barra colore (riferita all’intensità del rosso)
    colormap(gca, 'hot');           % solo per la colorbar (indicativa)
    cb = colorbar;
    cb.Label.String = '|\tau_{desired} - \tau_{exo}| (normalized)';
    caxis([0 1]);

catch ME
    warning('plotErrorPointCloud: fallback scatter: %s', ME.message);
    scatter3(P(:,1), P(:,2), P(:,3), opt.mSize, C, 'filled', 'MarkerEdgeColor','none', ...
             'MarkerFaceAlpha', mean(eNorm)*opt.alphaMax);
    axis equal; grid on; view(3);
    xlabel X; ylabel Y; zlabel Z; title([opt.titleStr ' (fallback)']);
end
end