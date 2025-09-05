% Function to analyze the patients, here the list of the patients and which
% arm they have affected:
% 06 healthy patient RIGHT handed.
% 23 stroke survivor patient LEFT affected.
function results = EAanalysis(Bod)

    analysisnumb = 1;

    % Ask for patient type
    patientType = '';
    while ~ismember(lower(patientType), {'h', 's'})
        patientType = lower(input('Analyze healthy (h) or stroke (s) patient? [h/s]: ', 's'));
        if ~ismember(patientType, {'h', 's'})
            disp('Invalid input. Please enter ''h'' or ''s''.');
        end
    end

    if patientType == 'h'
        folderName = fullfile('PatientsData', 'HealthyPatients');
    else
        % Stroke patient - ask BaseLine or FollowUp
        sessionType = '';
        while ~ismember(lower(sessionType), {'b', 'f'})
            sessionType = lower(input('Analyze BaseLine (b) or FollowUp (f)? [b/f]: ', 's'));
            if ~ismember(sessionType, {'b', 'f'})
                disp('Invalid input. Please enter ''b'' for BaseLine or ''f'' for FollowUp.');
            end
        end
    
        if sessionType == 'b'
            sessionFolder = 'BaseLine';
        else
            sessionFolder = 'FollowUp';
        end
    
        folderName = fullfile('PatientsData', 'StrokePatients', sessionFolder);
    end

    % Initialize results structure
    results.torque = {};   
    results.acceleration = {};
    results.velocity = {}; 
    results.filenames = {};

    % Choose files
    [filenames, isBatch] = chooseAdithFile(folderName);

    if isBatch
        % Batch mode: analyze all selected files
        for i = 1:length(filenames)
            filename = filenames{i};
            [time, Body] = readAdithFiles(filename);
            
            [torqueVal, accelVal, velVal ] = ShoulderDynamics(Body, time, Bod, filename);

            results.torque{analysisnumb} = torqueVal;
            results.acceleration{analysisnumb} = accelVal;
            results.velocity{analysisnumb} = velVal;
            results.filenames{analysisnumb} = filename;
            analysisnumb = analysisnumb + 1;
        end

        disp('Batch analysis completed.');

    else
        % Manual single-patient mode
        continueAnalysis = true;

        while continueAnalysis
            filename = filenames{1};
            [time, Body] = readAdithFiles(filename);

            [torqueVal, accelVal, velVal] = ShoulderDynamics(Body, time, Bod, filename);

            results.torque{analysisnumb} = torqueVal;
            results.acceleration{analysisnumb} = accelVal;
            results.velocity{analysisnumb} = velVal;
            results.filenames{analysisnumb} = filename;
            analysisnumb = analysisnumb + 1;

            % Ask if the user wants to analyze another subject
            repeatResponse = '';
            while ~ismember(lower(repeatResponse), {'y', 'n'})
                repeatResponse = input('\nDo you want to analyze another subject? (y/n): ', 's');
                if ~ismember(lower(repeatResponse), {'y', 'n'})
                    disp('Invalid input. Please enter ''y'' or ''n''.');
                end
            end

            if lower(repeatResponse) == 'n'
                continueAnalysis = false;
                disp('Analysis completed.');
            else
                [filenames, ~] = chooseAdithFile(folderName);
            end
        end
    end
end
