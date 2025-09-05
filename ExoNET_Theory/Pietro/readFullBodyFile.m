function [time, Body] = readFullBodyFile(filename)
% Reads full-body joint data from CSV and returns complete Body structure.

    if iscell(filename)
        filename = filename{1};
    end
    if ~isfile(filename)
        error('File "%s" not found.', filename);
    end
    fprintf('Reading file: %s\n', filename);

    data = readtable(filename);
    time = cellfun(@str2double, data{:,1});

    % === Full list of Kinect joints with order: X Z Y (as in your original)
    joints = {
        'SpineBase',     2;
        'SpineMid',      6;
        'Neck',         10;
        'Head',         14;
        'ShoulderLeft', 18;
        'ElbowLeft',    22;
        'WristLeft',    26;
        'HandLeft',     30;
        'ShoulderRight',34;
        'ElbowRight',   38;
        'WristRight',   42;
        'HandRight',    46;
        'HipLeft',      50;
        'KneeLeft',     54;
        'AnkleLeft',    58;
        'FootLeft',     62;
        'HipRight',     66;
        'KneeRight',    70;
        'AnkleRight',   74;
        'FootRight',    78;
        'SpineShoulder',82;
        'HandTipLeft',  86;
        'ThumbLeft',    90;
        'HandTipRight', 94;
        'ThumbRight',   98;
    };

    Body = struct();
    for i = 1:size(joints,1)
        jointName = joints{i,1};
        baseIdx   = joints{i,2};
        Body.(jointName) = [data{:, baseIdx}, data{:, baseIdx+2}, data{:, baseIdx+1}]; % X Z Y → X Y Z
    end
end
