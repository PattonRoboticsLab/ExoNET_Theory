function bestP_3D = loadBestP3D()
    % Get the folder where this function is located
    baseFolder = fileparts(mfilename('fullpath'));

    % Find subfolders (ignore . and ..)
    subFolders = dir(baseFolder);
    subFolders = subFolders([subFolders.isdir] & ~ismember({subFolders.name}, {'.', '..'}));

    if isempty(subFolders)
        error('No subfolders found inside the function directory.');
    end

    % Display subfolder list
    fprintf('Subfolders available:\n');
    for i = 1:length(subFolders)
        fprintf('%d: %s\n', i, subFolders(i).name);
    end

    % Let user select a folder
    choice = input('Select a subfolder by number: ');
    if choice < 1 || choice > length(subFolders)
        error('Invalid folder selection.');
    end

    % Build full path to selected folder
    selectedFolder = fullfile(baseFolder, subFolders(choice).name);
    matFile = fullfile(selectedFolder, 'bestP_3D.mat');

    % Check file existence
    if ~isfile(matFile)
        error('File "bestP_3D.mat" not found in the selected folder.');
    end

    % Load the variable from .mat file
    loadedData = load(matFile);

    if isfield(loadedData, 'bestP_3D')
        bestP_3D = loadedData.bestP_3D;
    else
        error('"bestP_3D" variable not found in the .mat file.');
    end
end
