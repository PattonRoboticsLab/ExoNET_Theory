function [bestP_3D, Torque_Exo, Torque_desired, Force_vector] = loadTorques()
    
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
    matFiles = dir(fullfile(selectedFolder, '*.mat'));

    if isempty(matFiles)
        error('No .mat files found in the selected folder.');
    end

    % Initialize output structures
    bestP_3D = [];
    Torque_Exo = struct('shoulder', [], 'elbow', []);
    Torque_desired = struct('shoulder', [], 'elbow', []);
    Force_vector = struct('exo_extended', [], 'desired_extended', [], ...
                           'exo_flexed',  [],  'desired_flexed',  []);

    % Loop through all .mat files to find required variables
    for i = 1:length(matFiles)
        fullPath = fullfile(selectedFolder, matFiles(i).name);
        try
            data = load(fullPath);
        catch
            warning('Skipping file "%s" (unable to load)', matFiles(i).name);
            continue;
        end

        % Extrapolate the parameters of the exo
        if isempty(bestP_3D) && isfield(data, 'bestP_3D')
            bestP_3D = data.bestP_3D;
        end

        % Torque exo
        if isfield(data, 'TauExo')
            if isfield(data.TauExo, 'elevationSh') && isempty(Torque_Exo.shoulder)
                Torque_Exo.shoulder = data.TauExo.elevationSh;
            end
            if isfield(data.TauExo, 'elevationEl') && isempty(Torque_Exo.elbow)
                Torque_Exo.elbow = data.TauExo.elevationEl;
            end
        end

        % Torque desired
        if isfield(data, 'TAUsDesired')
            if isfield(data.TAUsDesired, 'TauSh_tot') && isempty(Torque_desired.shoulder)
                Torque_desired.shoulder = data.TAUsDesired.TauSh_tot;
            end
            if isfield(data.TAUsDesired, 'TauEl_tot') && isempty(Torque_desired.elbow)
                Torque_desired.elbow = data.TAUsDesired.TauEl_tot;
            end
        end

        % Force vector fields
        if isfield(data, 'Force_vector')
            if isfield(data.Force_vector, 'exo_extended') && isempty(Force_vector.exo_extended)
                Force_vector.exo_extended = data.Force_vector.exo_extended;
            end
            if isfield(data.Force_vector, 'desired_extended') && isempty(Force_vector.desired_extended)
                Force_vector.desired_extended = data.Force_vector.desired_extended;
            end
            if isfield(data.Force_vector, 'exo_flexed') && isempty(Force_vector.exo_flexed)
                Force_vector.exo_flexed = data.Force_vector.exo_flexed;
            end
            if isfield(data.Force_vector, 'desired_flexed') && isempty(Force_vector.desired_flexed)
                Force_vector.desired_flexed = data.Force_vector.desired_flexed;
            end
        end
    end

    % Final check to see if the value are taken correctly
    if isempty(bestP_3D)
        error('Variable "bestP_3D" not found in any .mat file.');
    end
    if isempty(Torque_Exo.shoulder) || isempty(Torque_Exo.elbow)
        error('Torque_Exo fields missing.');
    end

    if isempty(Torque_desired.elbow)
        error('Torque_desired.elbow fields missing.');
    end
    
    if isempty(Torque_desired.shoulder) || isempty(Torque_desired.elbow)
        error('Torque_desired.shoulder fields missing.');
    end

    if isempty(Force_vector.exo_extended) || isempty(Force_vector.exo_flexed) || isempty(Force_vector.desired_extended) || isempty(Force_vector.desired_flexed)
        warning('Some Force_vector fields are missing.');
    end
end
