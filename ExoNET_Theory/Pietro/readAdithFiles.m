function [time, Body] = readAdithFiles(filename)
% readAdithFiles - Reads motion capture data from a CSV file and builds the Body structure.
%
% Inputs:
%   filename - Full path to the CSV file.
%
% Outputs:
%   time - Time vector (seconds).
%   Body - Structure containing selected body parts.

    % --- Check if the file exists ---
        % --- Handle cell input (select first if needed) ---
    if iscell(filename)
        filename = filename{1};
    end
    
    if ~isfile(filename)
        error('File "%s" not found.', filename);
    end

    if iscell(filename)
        fprintf('Reading file: %s\n', filename{1});
    else
        fprintf('Reading file: %s\n', filename);
    end

    % --- Read CSV file ---
    data = readtable(filename);

    % --- Extract Time (assuming first column) ---
    time = cellfun(@str2double, data{:,1});

    % --- Detect arm side from filename ---
    if contains(lower(filename), 'left')
        selectedArm = 'Left';
    elseif contains(lower(filename), 'right')
        selectedArm = 'Right';
    else
        error('Filename must contain "Left" or "Right" to detect the affected arm.');
    end

    % -------------------- Build Body structure ------------------------
    Body = struct();
    
    % Common body parts (present for both arms)
    Body.SpineBase     = [data{:, 2},  data{:, 4},  data{:, 3}];
    Body.SpineMid      = [data{:, 6},  data{:, 8},  data{:, 7}];
    Body.Neck          = [data{:, 10}, data{:, 12}, data{:, 11}];
    Body.Head          = [data{:, 14}, data{:, 16}, data{:, 15}];
    Body.SpineShoulder = [data{:, 82}, data{:, 84}, data{:, 83}];

    % --------------------- Arm-specific joints ------------------------
    if strcmp(selectedArm, 'Left')
        Body.Shoulder = [data{:, 18}, data{:, 20}, data{:, 19}];
        Body.Elbow    = [data{:, 22}, data{:, 24}, data{:, 23}];
        Body.Wrist    = [data{:, 26}, data{:, 28}, data{:, 27}]; 
        Body.Hand     = [data{:, 30}, data{:, 32}, data{:, 31}];
        Body.HandTip  = [data{:, 86}, data{:, 88}, data{:, 87}];
        Body.Thumb    = [data{:, 90}, data{:, 92}, data{:, 91}];

        Body.Shoulder_opposite = [data{:, 34}, data{:, 36}, data{:, 35}]; % Right shoulder
    
    else % Right arm
        Body.Shoulder = [data{:, 34}, data{:, 36}, data{:, 35}]; % Right shoulder
        Body.Elbow    = [data{:, 38}, data{:, 40}, data{:, 39}];
        Body.Wrist    = [data{:, 42}, data{:, 44}, data{:, 43}]; 
        Body.Hand     = [data{:, 46}, data{:, 48}, data{:, 47}];
        Body.HandTip  = [data{:, 94}, data{:, 96}, data{:, 95}];
        Body.Thumb    = [data{:, 98}, data{:, 100}, data{:, 99}];

        Body.Shoulder_opposite = [data{:, 18}, data{:, 20}, data{:, 19}]; % In this case is the left one
    end

end


