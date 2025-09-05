% Function to analyze the patients, here the list of the patients and which
% arm they have affected:
function results = EvaluationExoExpData(Bod, Exo, bestP_3D)

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
                disp('Invalid input. Please enter ''b'' or ''f''.');
            end
        end

        if sessionType == 'b'
            sessionFolder = 'BaseLine';
        else
            sessionFolder = 'FollowUp';
        end

        folderName = fullfile('PatientsData', 'StrokePatients', sessionFolder);
    end

    % Initialize results
    results.exo_torque = {};
    results.gvt_torque = {};
    results.torque_diff = {};
    results.filenames = {};

    % Choose files
    [filenames, isBatch] = chooseAdithFile(folderName);

    if isBatch
        % Batch mode: analyze all selected files
        for i = 1:length(filenames)
            filename = filenames{i};
            [time, Body] = readAdithFiles(filename);

            % Check arm side
            if contains(lower(filename), 'left')
                armSide = 'l';
            elseif contains(lower(filename), 'right')
                armSide = 'r';
            else
                error('Filename must contain "Left" or "Right" to detect affected side.');
            end

            %% Exo torque
            [TauExo, ~,~,~,~,~] = exoNetTorquesEvaluation(Body, Bod, Exo, bestP_3D, armSide);
            validRows = ~any(isnan([TauExo.elevationSh]), 2);
            Torque_elevation_sh_vector = TauExo.elevationSh(validRows,:);
            
            fs = 30;
            [b, a] = butter( 7, 10 / ( fs / 2 ) ); 
            Torque_elevation_sh_filt = filtfilt( b, a, Torque_elevation_sh_vector );
            Torque_elevation_sh_vector = movmean( Torque_elevation_sh_filt, 13, 'Endpoints', 'shrink');

            Torque_elevation_sh = vecnorm(Torque_elevation_sh_vector, 2, 2);
            
            nSamples = length(Torque_elevation_sh);
            dt = 1 / fs;
            time_vector = (0:nSamples-1) * dt;

            %% Torque from the patient
            [ ~, ~, torque ] = processArm(Body, time, Bod, armSide, filename);
  
            %% Fix the vector
            minLength = min([length(Torque_elevation_sh), length(torque.shoulder_gravity)]);
            
            gvt_torque = torque.shoulder_gravity(1:minLength); % GRavity torque from patient
            exo_torque = Torque_elevation_sh(1:minLength); % Torque from the exo

            torque_diff = gvt_torque - exo_torque; % Difference between the torques
            time_plot = time_vector(1:minLength);

            results.exo_torque{analysisnumb} = exo_torque;
            results.gvt_torque{analysisnumb} = gvt_torque;
            results.torque_diff{analysisnumb} = torque_diff;
            results.filenames{analysisnumb} = filename;
            analysisnumb = analysisnumb + 1;

            % Plotting
            figure('Name', 'Torque Comparison: Exo vs Shoulder Gravity');
            subplot(3,1,1);
            plot(time_plot, exo_torque, 'r'); ylabel('Torque (Nm)'); title('Torque from Exo'); grid on;
            subplot(3,1,2);
            plot(time_plot, gvt_torque, 'b'); ylabel('Torque (Nm)'); title('Gravity Torque'); grid on;
            subplot(3,1,3);
            plot(time_plot, torque_diff, 'k'); ylabel('Nm'); xlabel('Time (s)'); title('Difference (TauExo - Gravity)'); grid on;

            figure('Name', 'Torque Area Comparison','Color','w');
            h1 = area(time_plot, gvt_torque); h1.FaceColor = 'r'; hold on;
            h2 = area(time_plot, exo_torque); h2.FaceColor = 'b'; h2.FaceAlpha = 0.4;
            xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Area Plot: Gravity vs Exo Torque');
            legend('Gravity','Exo'); grid on;

            % Print summary
            fprintf('\n--- Average Torque Comparison [%s] ---\n', filename);
            fprintf('Average Gravity Torque:          %.2f Nm\n', mean(gvt_torque));
            fprintf('Average Exo Torque:              %.2f Nm\n', mean(exo_torque));
            fprintf('Average Difference:              %.2f Nm\n', mean(torque_diff));
            fprintf('Assisted Torque (%% of Gravity): %.2f%%\n',  mean((exo_torque ./ gvt_torque) * 100));
        end

        disp('Batch analysis completed.');

    else
        % Manual single-patient mode
        continueAnalysis = true;

        while continueAnalysis
            filename = filenames{1};
            [time, Body] = readAdithFiles(filename);

            if contains(lower(filename), 'left')
                armSide = 'l';
            elseif contains(lower(filename), 'right')
                armSide = 'r';
            else
                error('Filename must contain "Left" or "Right" to detect affected side.');
            end
            
           %% Exo torque
            [TauExo, ~,~,~,~,~] = exoNetTorquesEvaluation(Body, Bod, Exo, bestP_3D, armSide);
            validRows = ~any(isnan([TauExo.elevationSh]), 2);
            Torque_elevation_sh_vector = TauExo.elevationSh(validRows,:);
            
            fs = 30;
            [b, a] = butter( 7, 10 / ( fs / 2 ) ); 
            Torque_elevation_sh_filt = filtfilt( b, a, Torque_elevation_sh_vector );
            Torque_elevation_sh_vector = movmean( Torque_elevation_sh_filt, 13, 'Endpoints', 'shrink');

            Torque_elevation_sh = vecnorm(Torque_elevation_sh_vector, 2, 2);
            
            %% Torque from the patient
            [ ~, ~, torque ] = processArm(Body, time, Bod, armSide, filename);

            nSamples = length(Torque_elevation_sh);
            dt = 1 / fs;
            time_vector = (0:nSamples-1) * dt;

            minLength = min([length(Torque_elevation_sh), length(torque.shoulder_gravity)]);

            exo_torque = Torque_elevation_sh(1:minLength,:);
            gvt_torque = torque.shoulder_gravity(1:minLength,:);
            torque_diff = gvt_torque - exo_torque;
            time_plot = time_vector(1:minLength);

            results.exo_torque{analysisnumb} = exo_torque;
            results.gvt_torque{analysisnumb} = gvt_torque;
            results.torque_diff{analysisnumb} = torque_diff;
            results.filenames{analysisnumb} = filename;
            analysisnumb = analysisnumb + 1;

            % Plotting
            % figure('Name', 'Torque Comparison: Exo vs Shoulder Gravity');
            % subplot(3,1,1);
            % plot(time_plot, exo_torque, 'r'); ylabel('Torque (Nm)'); title('Torque from Exo'); grid on;
            % subplot(3,1,2);
            % plot(time_plot, gvt_torque, 'b'); ylabel('Torque (Nm)'); title('Gravity Torque'); grid on;
            % subplot(3,1,3);
            % plot(time_plot, torque_diff, 'k'); ylabel('Nm'); xlabel('Time (s)'); title('Difference (TauExo - Gravity)'); grid on;

            figure('Name', 'Torque Area Comparison','Color','w');
            h1 = area(time_plot, gvt_torque); h1.FaceColor = 'r'; hold on;
            h2 = area(time_plot, exo_torque); h2.FaceColor = 'b'; h2.FaceAlpha = 0.2;
            xlabel('Time (s)'); ylabel('Torque (Nm)'); title('Area Plot: Gravity vs Exo Torque');
            legend('Gravity','Exo'); grid on;

            % Print summary
            fprintf('\n--- Average Torque Comparison [%s] ---\n', filename);
            fprintf('Average Gravity Torque:          %.2f Nm\n', mean(gvt_torque));
            fprintf('Average Exo Torque:              %.2f Nm\n', mean(exo_torque));
            fprintf('Average Difference:              %.2f Nm\n', mean(torque_diff));
            fprintf('Assisted Torque (%% of Gravity): %.2f%%\n', mean((exo_torque ./ gvt_torque) * 100));
            
            %% Ask to see the animation of the patient with the exo
            animationResponse = '';
            while ~ismember(lower(animationResponse), {'y', 'n'})
                animationResponse = input('\nDo you want to see the animation? (y/n): ', 's');
                if ~ismember(lower(animationResponse), {'y', 'n'})
                    disp('Invalid input. Please enter ''y'' or ''n''.');
                end
            end

            if animationResponse == 'n'
                continueAnalysis = false;
                disp('Analysis completed.');
            else
                plotMovingArm( armSide, filename, Exo, bestP_3D);
            end

            %% Ask for another evaluation
            repeatResponse = '';
            while ~ismember(lower(repeatResponse), {'y', 'n'})
                repeatResponse = input('\nDo you want to analyze another subject? (y/n): ', 's');
                if ~ismember(lower(repeatResponse), {'y', 'n'})
                    disp('Invalid input. Please enter ''y'' or ''n''.');
                end
            end

            if repeatResponse == 'n'
                continueAnalysis = false;
                disp('Analysis completed.');
            else
                [filenames, ~] = chooseAdithFile(folderName);
            end

        end
    end
    
end
