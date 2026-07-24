function saveAllFigures()
% Salva tutte le figure (2D: 1 vista; 3D: 8 viste con elevazione 15°)
% Cartella: results_YYYYMMDD_N (se esiste già, incrementa N)
    
    dateStr    = datestr(now, 'yyyymmdd');
    baseFolderName = ['results_' dateStr];
    
    % Trova un nome di cartella non esistente
    folderName = fullfile(pwd, baseFolderName);
    counter = 1;
    while exist(folderName, 'dir')
        folderName = fullfile(pwd, sprintf('%s_%d', baseFolderName, counter));
        counter = counter + 1;
    end
    
    % Crea la cartella
    mkdir(folderName);
    
    figs = findall(0, 'Type', 'figure');
    if isempty(figs)
        disp('Nessuna figura aperta da salvare.'); 
        return;
    end
    
    hasExportGraphics = exist('exportgraphics','file') == 2;
    
    % Rotazione: 8 angoli da 0 a 360 in azimuth
    azList  = 0:45:315;    % azimuth
    elFixed = 15;          % elevazione dall'alto (15° invece di 0°)
    
    for f = 1:numel(figs)
        fig = figs(f); 
        figure(fig); 
        drawnow;
        
        axList = findall(fig, 'Type', 'axes');
        is3D = has3DContent(axList);
        
        if is3D
            set(fig, 'Renderer', 'opengl'); 
        end
        
        baseName = sprintf('figure_%02d', f);
        
        if ~is3D
            % Figura 2D: salva una sola vista
            outPng = fullfile(folderName, [baseName '.png']);
            safeSaveFigure(fig, outPng, hasExportGraphics);
            fprintf('Salvata (2D): %s\n', outPng);
        else
            % Figura 3D: salva 8 viste con elevazione 15°
            for az = azList
                for a = 1:numel(axList)
                    if isAxis3D(axList(a))
                        try
                            view(axList(a), [az, elFixed]);
                        catch
                            % Ignora errori
                        end
                    end
                end
                drawnow;
                outPng = fullfile(folderName, sprintf('%s_az%03d_el%02d.png', baseName, az, elFixed));
                safeSaveFigure(fig, outPng, hasExportGraphics);
                fprintf('Salvata (3D, az=%d°, el=%d°): %s\n', az, elFixed, outPng);
            end
        end
    end
    
    disp(['Tutte le figure salvate in: ' folderName]);
end

% ---------- Helper: rileva 3D ----------
function tf = has3DContent(axList)
    tf = false;
    for a = 1:numel(axList)
        if isAxis3D(axList(a))
            tf = true; 
            return; 
        end
    end
end

function tf = isAxis3D(ax)
    tf = false;
    if ~ishandle(ax) || ~strcmp(get(ax,'Type'),'axes')
        return; 
    end
    
    objs = findobj(ax, '-property', 'ZData');
    for k = 1:numel(objs)
        Z = get(objs(k), 'ZData');
        if ~isempty(Z)
            zvec = Z(:);
            if any(isfinite(zvec)) && (max(zvec) - min(zvec) ~= 0)
                tf = true; 
                return;
            end
        end
    end
    
    try
        [~, el] = view(ax);
        if abs(el - 90) > 1e-6
            tf = true; 
        end
    catch
        % Ignora errori
    end
end

% ---------- Helper: salvataggio ----------
function safeSaveFigure(fig, outFile, hasExportGraphics)
    if exist('exportgraphics','file') == 2 && hasExportGraphics
        exportgraphics(fig, outFile, 'Resolution', 300, 'BackgroundColor','white');
    else
        set(fig, 'Color', 'white');
        print(fig, outFile, '-dpng', '-r300');
    end
end
