%% Function for optimization of the parameters
function [Actual_Pin, collect_p, bestP_3D, bestCost, TauExo, L_springs, L0, Tension, Force_vector ] = robustOpto3D( Bod, Pos, Exo, PHIs, TAUsDesired, robot, q )

    %% Setup 
    fprintf('\n ~ RobustOpto ~ \n'); pause(.1); % Update display
    
    %% Number of iteration for fmincon
    num_samples = 1000;  % Number of iterations for the fmincon cycle
    
    %% Create some parameters for optimization
               ub = Exo.P(:, 2);         lb = Exo.P(:, 1);         % Set upper and lower bounds
          nparams = Exo.nParamsSh * round(Exo.numbconstraints(2)); % Evaluate numbers of parameters  
      indexswivel = nparams+1;                                     % Number of the parameter + 1 usefull to find the second row of parameters
        paramstot = length(Exo.P);                                 % Number of parameters   
    
    % Generazione punti iniziali con Latin Hypercube Sampling
    fprintf('Generando %d punti iniziali con Latin Hypercube Sampling...\n', num_samples);
    distributionp = lhsdesign(num_samples, paramstot);             % Values from 0 to 1 uniformely distributed to explore all the function
    
    %% Configuration fmincon
    optionsfmin = optimoptions('fmincon', ...
        'Display', 'iter', ...                  
        'Algorithm', 'interior-point', ...      
        'UseParallel', true, ...               
        'MaxFunctionEvaluations', 5000, ...     
        'SpecifyObjectiveGradient', true, ...
        'FiniteDifferenceStepSize', 1e-6, ...
        'MaxIterations', 300, ...               
        'OptimalityTolerance', 1e-5, ...         
        'FunctionTolerance', 1e-6, ...
        'StepTolerance', 1e-8, ...
        'EnableFeasibilityMode', true, ...
        'SubproblemAlgorithm', 'cg', ...
        'ScaleProblem', true);
    
    %% Setup parallel pool
    if isempty(gcp('nocreate'))
        parpool('local', min(feature('numcores'), num_samples));
    end
    
    fprintf('Iniziando ottimizzazione con %d campioni...\n\n', num_samples);            

    %% Loop multiple optimization
    fprintf('Begin optimizations:\n');   
    nTries = num_samples; % Set the first best cost, number of iteration  
    bestCost = 1000;   bestP_3D = [];  collect_p = [];  
    cost_function = @(x) cost(x, TAUsDesired, Exo, Pos, Bod, robot, q);

    for TRY = 1:nTries; fprintf('\nOpt#%d of %d\n', TRY, nTries);  % Start the optimization
    
        %% Fmincon
        % fprintf('\n... Using fmincon...')
        p0 = lb + (ub - lb) .* distributionp(TRY,:)'; % Vector containing the initial solution to optimize
        [p, new_cost] = fmincon(cost_function, p0, [], [], [], [], lb, ub, [], optionsfmin ); % use fmincon, core of the optimization
    
        %% Evaluation of the function
        if new_cost < bestCost   % If lower cost --> Update new cost and print below      
            fprintf(' Plot new solution: \n')
            fprintf(' c = %.3f ', new_cost ); 
            fprintf('\n p = \n'); 
            fprintf(' elemen    R     Theta      Y      L0     Perc      K     K_rec   L0_rec L_pre_rec \n')
            fprintf(' %.3f  ', p( 1 : indexswivel)');                fprintf('\n');
            fprintf(' %.3f  ', p( indexswivel+1 : indexswivel*2)' ); fprintf('\n');
            fprintf(' L0_wr  Perc_wr  K_wr \n');
            fprintf(' %.3f  ', p( indexswivel*2+1 : end)' );         fprintf('\n');        
            bestCost = new_cost;    bestP_3D = p;  
            collect_p = [collect_p; bestP_3D]; % Collect the best parameters, may be helpful
    
        else % else, not found a new best cost, print below
            fprintf(' The cost was: %g\n', new_cost);
            fprintf(' Not an improvement, still best cost = %g\n', bestCost);
            fprintf(' Parameter were:\n');
            fprintf(' elemen    R     Theta      Y      L0     Perc      K     K_rec   L0_rec L_pre_rec \n')
            fprintf(' %.3f  ', p(1:indexswivel)');               fprintf('\n');
            fprintf(' %.3f  ', p(indexswivel+1:indexswivel*2)'); fprintf('\n');
            fprintf(' L0_wr  Perc_wr  K_wr \n');
            fprintf(' %.3f  ', p(indexswivel*2+1:end)');         fprintf('\n');
            if exist('bestP_3D', 'var')
                fprintf(' \nBest parameters are:\n');
                fprintf(' elemen    R     Theta      Y      L0     Perc      K     K_rec   L0_rec L_pre_rec \n')
                fprintf(' %.3f  ', bestP_3D(1:indexswivel)');               fprintf('\n');
                fprintf(' %.3f  ', bestP_3D(indexswivel+1:indexswivel*2)'); fprintf('\n');
                fprintf(' L0_wr  Perc_wr  K_wr \n');
                fprintf(' %.3f  ', bestP_3D(indexswivel*2+1:end)');         fprintf('\n');
            end
        end     
    
        if TRY == nTries % Set the limit of optimization
           break
        end   
    end
    
    %% Final evaluation of the torques to store them into the final solution
    [TauExo, Actual_Pin, L_springs, L0, Tension, L0_recoil ] = exoNetTorques3D( Pos, Bod, Exo, bestP_3D, robot, q ); % Recall the functions to update the final torques with best cost
    
    %% Plot the values of the best solution
    fprintf(' \nParameters best solution: \n')
    fprintf(' c = %g ', bestCost)
    fprintf('\n                   R    Theta     Y      L0   Perc     K    K_rec  L0_rec L_pre_rec')
    for i = 1:round(bestP_3D( 1 ))
        fprintf('\n P_elevation %g = ', i); fprintf('%.3f  ', bestP_3D( Exo.nParamsSh*(i-1)+2 : Exo.nParamsSh*i+1 )' ); 
    end
    for i = 1:round(bestP_3D( nparams+2 ))
        fprintf('\n P_swivel %g    = ', i); fprintf('%.3f  ', bestP_3D( Exo.nParamsSh*(i-1)+2 + indexswivel : Exo.nParamsSh*i+1 + indexswivel)' ); 
    end
    fprintf('\n                   L0_wr  Perc_wr  K_wr  ')
    fprintf('\n P_elbow       = '); fprintf('%.3f   ', bestP_3D( end-2:end)' ); 
    
    %% Plot the vectors
    % fprintf('\nclear images blue\n'); hold off; hold off; delete(exograph1); delete(exograph2); delete(pin1); delete(pin2); 
        shoulder_pin = Exo.shoulder;                       center_back = [0, -0.146, 1.15];
    
         shoulder_el = [0.169316, -0.0167571, 1.39712];    shoulder_sw = [0.195279 -0.0285629 1.37362];
            elbow_el = [0.247853   0.221341  1.38841];        elbow_sw = [0.280967 0.207069 1.36473];
               wrist = [0.357447, 0.458538, 1.36375];         
    
    shoulder_el_flex = [0.141, 0.0382423, 1.30084];   shoulder_sw_flex = [0.185, -0.00145449, 1.37209];
       elbow_el_flex = [0.153,   0.172,   1.23023];      elbow_sw_flex = [0.182581 0.186 1.25767];
          wrist_flex = [-0.0652559, 0.195, 1.29763];         
    
    %% Figure with nancy body elbow extended
    figure('Name','Error field elbow extended'); nancy_body; hold on; title('0° internal rotation / 0° elbow flexion');
    legend4 = PlotPin( Exo, Actual_Pin, shoulder_pin, shoulder_el, shoulder_sw, elbow_sw, elbow_el, bestP_3D, center_back, wrist, L0_recoil );
    Force_vector.exo_extended = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TauExo.elevationSh, TauExo.elevationEl, ...
                    [0, 0, 1], 3,   4, 2, 'Transparency', 0.7);
    Force_vector.desired_extended = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TAUsDesired.TauSh_tot,  TAUsDesired.TauEl_tot, ...
                    [1, 0, 0], 3,   4, 2, 'IsDesired', true, ...
                    'Transparency', 0.9, 'Offset', [0.001, 0.001, 0.001]); hold off; % grid off; legend(legend4);
    
    %% Figure with nancy body elbow flexed 
    figure('Name','Error field elbow flexed'); nancy_body_flex; hold on; title('90° internal rotation / 90° elbow flexion');
    legend5 = PlotPin( Exo, Actual_Pin, shoulder_pin, shoulder_el_flex, shoulder_sw_flex, elbow_sw_flex, elbow_el_flex, bestP_3D, center_back, wrist_flex, L0_recoil );
    Force_vector.exo_flexed = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TauExo.elevationSh, TauExo.elevationEl, ...
                    [0, 0, 1], 3,   4, 1, 'Transparency', 0.7);
    Force_vector.desired_flexed = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TAUsDesired.TauSh_tot, TAUsDesired.TauEl_tot, ...
                    [1, 0, 0], 3,   4, 1, 'IsDesired', true, ...
                    'Transparency', 0.9, 'Offset', [0.001, 0.001, 0.001]); hold off; % grid off; legend(legend5); 
 

    %% Figure only pins
    figure('Name','Error field elbow flexed'); hold on; title('90° internal rotation / 90° elbow flexion');
    legend6 = PlotPin( Exo, Actual_Pin, shoulder_pin, shoulder_el, shoulder_sw, elbow_sw, elbow_el, bestP_3D, center_back, wrist, L0_recoil );
    hold off; %legend(legend6); 
    
    %% Plot the torque for the shoulder for first plane of elevation (0°)
    Tau_desired_sh_firstplane = vecnorm( TAUsDesired.TauSh_tot(1:2:end,:), 2, 2 );
        Tau_exo_sh_firstplane = vecnorm( TauExo.elevationSh(1:2:end,:), 2, 2 );
    timetorque = 0 : length(Tau_desired_sh_firstplane) - 1;
    
    figure('Name', 'Torque Comparison: Exo vs Shoulder Gravity'); hold on; title('0° internal rotation / 0° elbow flexion');
    scatter(timetorque, Tau_exo_sh_firstplane,     'r'); 
    scatter(timetorque, Tau_desired_sh_firstplane, 'b');
    xlabel('Frames');  ylabel('Torque (Nm)'); legend('Exoskeleton Torque', 'Desired Shoulder Torqsue');
    grid on;
    
    %% Plot the torque for the shoulder for second plane of elevation (90°)
    Tau_desired_sh_secondplane = vecnorm( TAUsDesired.TauSh_tot(2:2:end,:), 2, 2 );
        Tau_exo_sh_secondplane = vecnorm( TauExo.elevationSh(2:2:end,:), 2, 2 );
    timetorque = 0:length(Tau_desired_sh_secondplane)-1;
    
    figure('Name', 'Torque Comparison: Exo vs Shoulder Gravity'); hold on; 
    title('90° internal rotation / 90° elbow flexion');
    scatter(timetorque, Tau_exo_sh_secondplane,     'r'); 
    scatter(timetorque, Tau_desired_sh_secondplane, 'b');
    xlabel('Frames');  ylabel('Torque (Nm)');  legend('Exoskeleton Torque', 'Desired Shoulder Torque');
    grid on;

end