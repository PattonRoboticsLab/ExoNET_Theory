% Function to read the file inside the folder and decide what arm to
% evaluate of the patient
function [filenames, isBatch] = chooseAdithFile(folder)
% chooseAdithFile - Allows user to select one or multiple patient files.
% 
% Inputs:
%   folder - Base folder containing 'Left' and 'Right' subfolders.
%
% Outputs:
%   filenames - Cell array of selected file paths.
%   isBatch   - Logical flag indicating batch mode.

    isBatch = false; % Default is single file mode

    % Ask user: analyze all patients or one?
    choice = '';
    while ~ismember(lower(choice), {'a', 'o'})
        choice = lower(input('Analyze (a)ll patients or (o)ne specific? [a/o]: ', 's'));
        if ~ismember(choice, {'a', 'o'})
            disp('Invalid input. Please enter ''a'' for all or ''o'' for one.');
        end
    end

    if choice == 'a'
        % Batch mode: collect all CSV files from both arms
        isBatch = true;
        filenames = {};

        arms = {'Left', 'Right'};
        for i = 1:2
            armFolder = fullfile(folder, arms{i});
            filePattern = fullfile(armFolder, '*.csv');
            csvFiles = dir(filePattern);

            for k = 1:numel(csvFiles)
                filenames{end+1} = fullfile(armFolder, csvFiles(k).name); %#ok<AGROW>
            end
        end

        fprintf('\nFound %d files in "%s\\Left" and "%s\\Right".\n', numel(filenames), folder, folder);

    else
        % Single-patient manual mode
        arm = '';
        while ~ismember(lower(arm), {'l', 'r'})
            arm = lower(input('Which arm is affected? (l/r): ', 's'));
            if ~ismember(arm, {'l', 'r'})
                disp('Invalid input. Please enter ''l'' or ''r''.');
            end
        end

        if arm == 'l'
            armFolder = fullfile(folder, 'Left');
        else
            armFolder = fullfile(folder, 'Right');
        end

        filePattern = fullfile(armFolder, '*.csv');
        csvFiles = dir(filePattern);

        if isempty(csvFiles)
            error('No CSV files found in "%s".', armFolder);
        end

        % List available files
        fprintf('\nFiles found in "%s":\n', armFolder);
        for k = 1:numel(csvFiles)
            fprintf('%d: %s\n', k, csvFiles(k).name);
        end

        % Ask user to select
        fileIndex = input('Enter the number of the file you want to select: ');
        if fileIndex < 1 || fileIndex > numel(csvFiles)
            error('Invalid file selection.');
        end

        filenames = { fullfile(armFolder, csvFiles(fileIndex).name) };
    end
end
