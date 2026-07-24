function [p, c, TAUs, Exo, F] = loadBestPartha()
% LOADBESTPARTHA Carica un set di risultati salvato in "best Partha"
%
%   [p, c, TAUs, Exo] = loadBestPartha()
%
%   - Ti mostra un elenco dei file presenti nella cartella "best Partha"
%   - Scegli quale caricare
%   - Restituisce le variabili salvate in quel file:
%       p, c, TAUs, Exo
%
%   ⚠️ Assicura che i file siano stati salvati con saveBestPartha.m

    folderName = 'best Partha';

    % Verifica che la cartella esista
    if ~exist(folderName, 'dir')
        error('❌ La cartella "%s" non esiste.', folderName);
    end

    % Trova tutti i file .mat nella cartella
    files = dir(fullfile(folderName, '*.mat'));

    if isempty(files)
        error('❌ Nessun file .mat trovato nella cartella "%s".', folderName);
    end

    % Se c'è più di un file → selezione interattiva
    if numel(files) > 1
        fprintf('📂 File disponibili in "%s":\n', folderName);
        for i = 1:numel(files)
            fprintf('  [%d] %s\n', i, files(i).name);
        end

        idx = input('👉 Seleziona il numero del file da caricare: ');
        if isempty(idx) || idx < 1 || idx > numel(files)
            error('❌ Selezione non valida.');
        end
    else
        idx = 1;
    end

    % Nome file selezionato
    fileToLoad = fullfile(folderName, files(idx).name);

    % Carica le variabili
    S = load(fileToLoad, 'p', 'c', 'TAUs', 'Exo','F','');

    % Assegna alle variabili di output
    p    = S.p;
    c    = S.c;
    TAUs = S.TAUs;
    Exo  = S.Exo;
    F    = S.F;
    fprintf('✅ File "%s" caricato correttamente.\n', files(idx).name);
end
