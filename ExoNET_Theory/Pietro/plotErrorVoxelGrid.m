function plotErrorVoxelGrid(Pos, Exo, err_vec, varargin)
% plotErrorVoxelGrid(Pos, err_vec, 'which','wrist'|'elbow', ...)
% - Visualizza SOLO i voxel che contengono punti.
% - Colori: blu (<0) → bianco (=0) → rosso (>0), centro ESATTO su 0.
% - Trasparenza: |errore|=0 ⇒ alpha=0 (invisibile); cresce fino ad alphaMax
%   quando |errore| >= alphaRef (senza normalizzazione globale).

% ---------- Parametri ----------
p = inputParser;
addParameter(p,'which','wrist');         % 'wrist'|'elbow'
addParameter(p,'nx',12); addParameter(p,'ny',12); addParameter(p,'nz',10);
addParameter(p,'alphaMax', 2);           % opacità massima
addParameter(p,'alphaMin', 0.0);         % opacità minima (default 0 = com’era)
addParameter(p,'alphaScale','linear');   % 'linear'|'sqrt'|'log'
addParameter(p,'alphaRef',1.0);          % |errore| che mappa ad alphaMax
addParameter(p,'smoothSigma',0);         % smoothing in voxel (0=off)
addParameter(p,'titleStr','Voxel grid - error centered at 0');
addParameter(p,'shiftVec',[0 0 0]);     % vettore di traslazione uniforme [dx dy dz]
addParameter(p,'snapCenterTo',[]);      % [x y z] target dove portare il centro griglia
addParameter(p,'snapToTag','');         % se dato, usa il centroid del patch con questo Tag

parse(p,varargin{:}); opt = p.Results;

% ---------- Dati ----------
switch lower(opt.which)
    case 'wrist', P = Pos.wrSwivel;
    case 'elbow', P = Pos.elbowSwivel;
    otherwise, error('Parameter "which" must be "wrist" or "elbow".');
end
N = min(size(P,1), numel(err_vec));
P = P(1:N,:); E = err_vec(1:N);  % errori CON SEGNO (unità reali, es. Nm)

% ---------- Riferimento rispetto a Pos.sh ----------
if isfield(Pos,'sh') && ~isempty(Pos.sh)
    S = Exo.shoulder - [0,0,1.4];
    % Porta S in formato Nx3 compatibile con P
    if isvector(S) && numel(S)==3
        S = repmat(reshape(S,1,3), N, 1);
    elseif size(S,2)==3 && size(S,1) >= N
        S = S(1:N,:);
    else
        error('Pos.sh deve essere [1x3] o [Nx3].');
    end
    % sposta i punti: coordinate relative alla spalla
    P = P + S;
else
    warning('Pos.sh mancante o vuoto: uso coordinate assolute.');
end

% ---------- Bordi griglia ----------
pad = 1e-9;
xmin = min(P(:,1))-pad; xmax = max(P(:,1))+pad;
ymin = min(P(:,2))-pad; ymax = max(P(:,2))+pad;
zmin = min(P(:,3))-pad; zmax = max(P(:,3))+pad;

xedges = linspace(xmin, xmax, opt.nx+1);
yedges = linspace(ymin, ymax, opt.ny+1);
zedges = linspace(zmin, zmax, opt.nz+1);
nx = numel(xedges)-1; ny = numel(yedges)-1; nz = numel(zedges)-1;

% ---------- Assegna punti a voxel ----------
ix = discretize(P(:,1), xedges);
iy = discretize(P(:,2), yedges);
iz = discretize(P(:,3), zedges);
valid = ~(isnan(ix)|isnan(iy)|isnan(iz));

% ---------- Aggrega errori per voxel (media) ----------
voxelData = containers.Map('KeyType','char','ValueType','any');
for i = 1:N
    if ~valid(i), continue; end
    ii = ix(i); jj = iy(i); kk = iz(i);
    if ii<1||ii>nx||jj<1||jj>ny||kk<1||kk>nz, continue; end
    key = sprintf('%d_%d_%d', ii,jj,kk);
    if isKey(voxelData,key)
        voxelData(key) = [voxelData(key), E(i)];
    else
        voxelData(key) = E(i);
    end
end
if isempty(voxelData), error('Nessun punto trovato nei voxel!'); end

keys = voxelData.keys; nv = numel(keys);
voxelIdx = zeros(nv,3);
voxelErr = zeros(nv,1);
voxelAlp = zeros(nv,1);

for v = 1:nv
    ijk = str2double(strsplit(keys{v},'_'));
    voxelIdx(v,:) = ijk;
    m = mean(voxelData(keys{v}));
    voxelErr(v) = m;

    % --- Alpha da |errore|, con curva scelta ---
    t = abs(m)/opt.alphaRef;          % riferimento ASSOLUTO
    switch lower(opt.alphaScale)
        case 'sqrt', t = sqrt(t);
        case 'log',  t = log1p(t);
        otherwise    % linear
    end
    t = min(t,1);
    voxelAlp(v) = max(opt.alphaMin, opt.alphaMax * t);

end

% ---------- Smoothing opzionale ----------
if opt.smoothSigma > 0
    Evol = zeros(nx,ny,nz);
    for v = 1:nv
        ii = voxelIdx(v,1); jj = voxelIdx(v,2); kk = voxelIdx(v,3);
        Evol(ii,jj,kk) = voxelErr(v);
    end
    g = local_gaussian1d(opt.smoothSigma);
    Evol = convn(Evol, reshape(g,[],1,1),'same');
    Evol = convn(Evol, reshape(g,1,[],1),'same');
    Evol = convn(Evol, reshape(g,1,1,[]),'same');

    for v = 1:nv
        ii = voxelIdx(v,1); jj = voxelIdx(v,2); kk = voxelIdx(v,3);
        voxelErr(v) = Evol(ii,jj,kk);

        t = abs(voxelErr(v))/opt.alphaRef;
        switch lower(opt.alphaScale)
            case 'sqrt', t = sqrt(t);
            case 'log',  t = log1p(t);
            otherwise
        end
        t = min(t,1);
        voxelAlp(v) = opt.alphaMax * t;
    end
end

% ---------- Geometria patch SOLO per voxel con dati ----------
V0 = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1];
F0 = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

V = zeros(nv*8,3); F = zeros(nv*6,4);
S = zeros(nv*8,1); Av = zeros(nv*8,1);
vptr = 0; fptr = 0;

for v = 1:nv
    ii = voxelIdx(v,1); jj = voxelIdx(v,2); kk = voxelIdx(v,3);
    x0 = xedges(ii);   x1 = xedges(ii+1);
    y0 = yedges(jj);   y1 = yedges(jj+1);
    z0 = zedges(kk);   z1 = zedges(kk+1);

    Vcube = V0;
    Vcube(:,1) = Vcube(:,1)*(x1-x0) + x0;
    Vcube(:,2) = Vcube(:,2)*(y1-y0) + y0;
    Vcube(:,3) = Vcube(:,3)*(z1-z0) + z0;

    V(vptr+(1:8),:) = Vcube;
    F(fptr+(1:6),:) = F0 + vptr;

    S(vptr+(1:8))  = voxelErr(v);   % stesso valore per tutti i vertici del cubo
    Av(vptr+(1:8)) = voxelAlp(v);   % stessa alpha per tutti i vertici

    vptr = vptr + 8; fptr = fptr + 6;
end
% ===== Traslazione uniforme / Ricentraggio voxel =====
% Centro attuale della griglia (in base ai bordi calcolati)
gridCenter = [(xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2];

% Se richiesto, ricava automaticamente il target dal Tag (es. 'nancy_body')
if ~isempty(opt.snapToTag) && isempty(opt.snapCenterTo)
    h = findobj('Type','patch','Tag',opt.snapToTag);
    if ~isempty(h) && isprop(h(1),'Vertices') && ~isempty(h(1).Vertices)
        tgt = mean(h(1).Vertices,1,'omitnan');     % centroid del patch
        opt.snapCenterTo = tgt;
    end
end

% Determina l’offset finale
if ~isempty(opt.snapCenterTo)
    offset = opt.snapCenterTo - gridCenter;   % porta il centro griglia sul target
else
    offset = opt.shiftVec;                    % semplice traslazione lungo un vettore
end

% Applica l’offset a TUTTI i vertici dei voxel (visualizzazione)
if any(offset~=0)
    V(:,1) = V(:,1) + offset(1);
    V(:,2) = V(:,2) + offset(2);
    V(:,3) = V(:,3) + offset(3);
end

% ---------- Colori centrati su 0 (zero = bianco) ----------
cmap = make_custom_bwr(256);
emax = max(abs(S)); if emax <= eps, emax = 1; end
midIdx = ceil(size(cmap,1)/2);
scale  = (size(cmap,1)-1)/2;
idx = round((S/emax)*scale) + midIdx;      % -emax→1, 0→mid, +emax→end
idx = max(1, min(size(cmap,1), idx));      % clamp
Cvert = cmap(idx,:);

% ---------- Plot ----------
set(gcf,'Renderer','opengl','Color','w');
ax = gca; hold(ax,'on'); set(ax,'Color',[0.95 0.95 0.95]); set(ax,'SortMethod','depth');

patch('Vertices',V,'Faces',F, ...
      'FaceColor','interp', ...
      'FaceVertexCData',Cvert, ...
      'FaceAlpha','interp', ...
      'FaceVertexAlphaData',Av, ...
      'AlphaDataMapping','none', ...
      'EdgeColor','k','EdgeAlpha',0.08,'LineWidth',0.1, ...
      'FaceLighting','gouraud','SpecularStrength',0.3, ...
      'Tag','voxel_patch');

axis equal tight vis3d; grid on; box on;
xlabel('Frontal plane'); ylabel('Sagital plane'); zlabel('Trasverse plane');
title(opt.titleStr);
view(45,30); rotate3d on;
%camlight('headlight'); camlight('right'); camlight('left'); lighting gouraud;

% ---------- Colorbar (non altera i colori del patch) ----------
% ---------- Colorbar NORMALE senza toccare Nancy ----------
emax = max(abs(S(:))); if emax <= eps, emax = 1; end

% Axes invisibile dedicato alla colorbar (così colormap/clim non toccano Nancy)
axCB = axes('Parent', gcf, ...
            'Position', get(gca,'Position'), ...
            'Visible','off', 'HitTest','off');

% Colormap e limiti SOLO su axCB (NON sul gca di Nancy)
colormap(axCB, cmap);                 % usa la tua cmap BWR
try
    clim(axCB, [-emax, emax]);        % R2022a+; centra lo zero
catch
    caxis(axCB, [-emax, emax]);       % fallback versioni più vecchie
end

% Colorbar normale agganciata ad axCB
c_sh = colorbar(axCB);
c_sh.Label.String     = 'Error torque Nm';
c_sh.Label.FontSize   = 12;
c_sh.Label.FontWeight = 'bold';

% Allinea la colorbar a destra dell'axes corrente (quello dove hai plottato)
posAx = get(gca,'Position');
posCB = get(c_sh,'Position');
posCB(1) = posAx(1) + posAx(3) + 0.02;   % sposta a destra
posCB(2) = posAx(2);
posCB(4) = posAx(4);
set(c_sh, 'Position', posCB);


% ---------- Diagnostica ----------
fprintf('--- VOXEL GRID ---\n');
fprintf('Voxel mostrati: %d / %d\n', nv, nx*ny*nz);
fprintf('Errore min/max (per-vertex S): [%.4g, %.4g]\n', min(S), max(S));
fprintf('Alpha min/max: [%.3f, %.3f] (alphaRef=%.3f)\n', min(Av), max(Av), opt.alphaRef);

end

% ================== Helper ==================
function g = local_gaussian1d(sigma)
if sigma<=0, g=1; return; end
half = max(1, ceil(3*sigma));
x = -half:half; g = exp(-(x.^2)/(2*sigma^2)); g = g/sum(g);
end

function cmap = make_custom_bwr(n)
% Blu→Bianco→Rosso, centro ESATTO su bianco
if nargin==0, n=256; end
half = floor(n/2);
bw = [linspace(0,1,half)', linspace(0,1,half)', ones(half,1)];
wr = [ones(half,1), linspace(1,0,half)', linspace(1,0,half)'];
cmap = [bw; wr];
if size(cmap,1) < n
    cmap = [cmap(1:half,:); 1 1 1; cmap(half+1:end,:)];
end
end
