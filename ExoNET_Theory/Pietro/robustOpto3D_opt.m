%% Function for optimization of the parameters
function [Actual_Pin, bestP_3D, bestCost, TauExo, L_springs, L0, Tension, Force_vector ] = robustOpto3D_opt( Bod, Pos, Exo, PHIs, TAUsDesired, robot, q )

    %% Setup 
    fprintf('\n ~ RobustOpto ~ \n'); pause(.1); % Update display
    
    pool = gcp('nocreate'); 
    if isempty(pool)
        pool = parpool; 
    end 

    %% === Setup bounds and parameters ===
    fprintf('📊 Configurazione parametri...\n');
    try
        ub = Exo.P(:, 2);   lb = Exo.P(:, 1);
        
        % Validazione bounds
        assert(length(lb) == length(ub), 'Dimensioni bounds non compatibili');
        assert(all(lb <= ub), 'Bounds non validi: lb deve essere <= ub');
        
        nparams = Exo.nParamsSh * round(Exo.numbconstraints(2));
        indexswivel = nparams + 1;
        paramstot = length(Exo.P);
        
        fprintf('   Parametri totali: %d\n', paramstot);
        
    catch ME
        error('Errore nella configurazione parametri: %s', ME.message);
    end
    
    %% Define cost function with error handling 
    cost_function = @(x) cost(x, TAUsDesired, Exo, Pos, Bod, robot, q);
    
    %% Initialize tracking variables
    optimization_costs = zeros(3, 1);    optimization_times = zeros(3, 1);
    
    %% ======================== GENETIC ALGORITHM =========================
    fprintf('\n🔍 FASE 1: Genetic Algorithm...\n');
    opts_ga = optimoptions('ga', ...
        'Display', 'iter', ...
        'UseParallel', true, ...
        'PopulationSize', 2500, ...
        'MaxGenerations', 300, ...
        'EliteCount', 30, ...
        'CrossoverFraction', 0.85, ...
        'FunctionTolerance', 1e-3, ...
        'CrossoverFcn', @crossoverscattered, ...
        'MutationFcn', @mutationgaussian);
    
    tic;

    intcon = 1; 
    [p_ga, cost_ga, exitflag_ga, output_ga] = ga(cost_function, paramstot, [], [], [], [], lb, ub, [], intcon, opts_ga);
    
    optimization_costs(1) = cost_ga;     optimization_times(1) = toc;
    
    fprintf('   ✅ GA completato: Costo = %.6f | Generazioni = %d | Tempo = %.2fs\n', ...
        cost_ga, output_ga.generations, optimization_times(1));
    
    if exitflag_ga <= 0
        warning('GA terminato con exitflag = %d', exitflag_ga);
    end
    
    %% =====================  SIMULATED ANNEALING =========================
    fprintf('\n❄️ FASE 2: Simulated Annealing...\n');
    opts_sa = optimoptions('simulannealbnd', ...
        'Display', 'iter', ...
        'MaxIterations', 300, ...
        'FunctionTolerance', 1e-4);
    tic;

    [p_sa, cost_sa, exitflag_sa, output_sa] = simulannealbnd(cost_function, p_ga, lb, ub, opts_sa);
    
    optimization_costs(2) = cost_sa;  optimization_times(2) = toc;
    
    fprintf('   ✅ SA completato: Costo = %.6f | Iterazioni = %d | Tempo = %.2fs\n', ...
        cost_sa, output_sa.iterations, optimization_times(2));
    
    if exitflag_sa <= 0
        warning('SA terminato con exitflag = %d', exitflag_sa);
    end

    %% ==================== FMINCON LOCAL OPTIMIZATION ====================
    fprintf('\n🎯 FASE 3: Ottimizzazione locale (fmincon)...\n');
    opts_fmincon = optimoptions('fmincon', ...
        'Display', 'iter', ...
        'Algorithm', 'interior-point', ...
        'UseParallel', true, ...
        'SpecifyObjectiveGradient', false, ... % Cambiato per sicurezza
        'MaxFunctionEvaluations', 5000, ...
        'MaxIterations', 300, ...
        'FiniteDifferenceStepSize', 1e-6, ...
        'OptimalityTolerance', 1e-5, ...
        'FunctionTolerance', 1e-6, ...
        'StepTolerance', 1e-8, ...
        'EnableFeasibilityMode', true, ...
        'SubproblemAlgorithm', 'cg', ...
        'ScaleProblem', true);
    
    tic;
    
    [p_final, bestCost, exitflag_fmincon, output_fmincon] = fmincon(cost_function, p_sa, [], [], [], [], ...
                                                                    lb, ub, [], opts_fmincon);
    
    optimization_costs(3) = bestCost;
    optimization_times(3) = toc;
    
    fprintf('   ✅ fmincon completato: Costo = %.6f | Iterazioni = %d | Tempo = %.2fs\n', ...
        bestCost, output_fmincon.iterations, optimization_times(3));
    
    if exitflag_fmincon <= 0
        warning('fmincon terminato con exitflag = %d', exitflag_fmincon);
    end            

    bestP_3D = p_final; % Assign final result 
    
    %% Results summary 
    fprintf('\n📈 RIEPILOGO OTTIMIZZAZIONE:\n');
    fprintf('   GA:      Costo = %.6f | Tempo = %.2fs\n', optimization_costs(1), optimization_times(1));
    fprintf('   SA:      Costo = %.6f | Tempo = %.2fs\n', optimization_costs(2), optimization_times(2));
    fprintf('   fmincon: Costo = %.6f | Tempo = %.2fs\n', optimization_costs(3), optimization_times(3));
    fprintf('   Tempo totale: %.2fs\n', sum(optimization_times));
    
    % Calcolo miglioramento
    if optimization_costs(1) > 0
        improvement = (optimization_costs(1) - bestCost) / optimization_costs(1) * 100;
        fprintf('   Miglioramento: %.2f%%\n', improvement);
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
                                                    [0, 0, 1], 3,   4, 1, 'Transparency', 0.7);
    Force_vector.desired_extended = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TAUsDesired.TauSh_tot,  TAUsDesired.TauEl_tot, ...
                                                    [1, 0, 0], 3,   4, 1, 'IsDesired', true, ...
                                                    'Transparency', 0.9, 'Offset', [0.001, 0.001, 0.001]); hold off; % grid off; legend(legend4);
                                    
    %% Figure with nancy body elbow flexed 
    figure('Name','Error field elbow flexed'); nancy_body_flex; hold on; title('90° internal rotation / 90° elbow flexion');
    legend5 = PlotPin( Exo, Actual_Pin, shoulder_pin, shoulder_el_flex, shoulder_sw_flex, elbow_sw_flex, elbow_el_flex, bestP_3D, center_back, wrist_flex, L0_recoil );
    Force_vector.exo_flexed = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TauExo.elevationSh, TauExo.elevationEl, ...
                    [0, 0, 1], 3,   4, 2, 'Transparency', 0.7);
    Force_vector.desired_flexed = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TAUsDesired.TauSh_tot, TAUsDesired.TauEl_tot, ...
                    [1, 0, 0], 3,   4, 2, 'IsDesired', true, ...
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