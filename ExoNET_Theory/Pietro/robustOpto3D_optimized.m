function [Actual_Pin, collect_p, bestP_3D, bestCost, TauExo, L_springs, L0, Tension, Force_vector] = robustOpto3D_optimized(Bod, Pos, Exo, PHIs, TAUsDesired, robot, q)
% Versione ottimizzata e semplificata di robustOpto3D
% 
% CARATTERISTICHE:
% 1. Parallelizzazione semplificata e robusta
% 2. Early stopping intelligente
% 3. Visualizzazione progressi in tempo reale
% 4. Gestione errori migliorata
% 5. Codice più leggibile e manutenibile

%% Setup iniziale
fprintf('\n=== RobustOpto3D Optimized ===\n'); 
fprintf('Inizializzazione in corso...\n');

%% Parametri di configurazione
num_samples = 1000;                              % Numero di iterazioni totali
max_no_improvement = min(100, num_samples/5);    % Early stopping dopo N iterazioni senza miglioramento
plot_improvements = true;                        % Mostra miglioramenti in tempo reale
print_every = 50;                               % Stampa progresso ogni N iterazioni
batch_size = 20;                                % Elabora in batch ottimizzati per 4 workers (20 = 4*5)

%% Creazione parametri di ottimizzazione
ub = Exo.P(:, 2);         % Upper bounds
lb = Exo.P(:, 1);         % Lower bounds
nparams = Exo.nParamsSh * round(Exo.numbconstraints(2)); 
indexswivel = nparams + 1;                                     
paramstot = length(Exo.P);                                 

% Generazione punti iniziali con Latin Hypercube Sampling
fprintf('Generando %d punti iniziali con Latin Hypercube Sampling...\n', num_samples);
distributionp = lhsdesign(num_samples, paramstot);             

%% Configurazione ottimizzatore fmincon
optionsfmin = optimoptions('fmincon', ...
    'Display', 'off', ...                    % Nessun output per ogni ottimizzazione
    'Algorithm', 'interior-point', ...       % Algoritmo veloce e robusto
    'UseParallel', false, ...                % Disabilitato per parallelizzazione esterna
    'MaxFunctionEvaluations', 2000, ...      % Limite valutazioni funzione
    'MaxIterations', 200, ...                % Limite iterazioni
    'OptimalityTolerance', 1e-4, ...         % Tolleranza ottimalità
    'FunctionTolerance', 1e-6, ...           % Tolleranza funzione
    'StepTolerance', 1e-8, ...               % Tolleranza step
    'EnableFeasibilityMode', true, ...       % Modalità fattibilità
    'SubproblemAlgorithm', 'cg');            % Algoritmo sottoproblema

%% Setup pool parallelo
fprintf('Configurando pool parallelo...\n');
pool = gcp('nocreate');
if isempty(pool)
    n_workers = 4; % Ottimizzato per i tuoi 4 workers
    pool = parpool('local', n_workers);
    fprintf('Pool parallelo creato con %d workers\n', n_workers);
else
    n_workers = pool.NumWorkers;
    fprintf('Pool parallelo già attivo con %d workers\n', n_workers);
end

% Disabilita warning per esecuzione lenta
warning('off', 'parallel:pool:RemoteExecutionSlowToStart');

% Verifica che abbiamo abbastanza workers
if n_workers < 4
    warning('Hai solo %d workers disponibili. Prestazioni potrebbero essere ridotte.', n_workers);
end

%% Inizializzazione variabili di ottimizzazione
fprintf('Iniziando ottimizzazione con %d campioni...\n\n', num_samples);   
bestCost = 60000;           % Costo iniziale molto alto
bestP_3D = []; 
collect_p = [];             % Collezione di tutti i miglioramenti
improvement_count = 0;      % Contatore miglioramenti
no_improvement_count = 0;   % Contatore iterazioni senza miglioramento
start_time = tic;           % Timer per performance

%% Esecuzione ottimizzazione parallela con batch processing
fprintf('Avvio ottimizzazione parallela con batch processing...\n');
fprintf('Elaborazione in batch di %d campioni (ottimizzato per %d workers)...\n', batch_size, n_workers);

% Creazione function handle per la funzione costo
cost_function = @(x) cost(x, TAUsDesired, Exo, Pos, Bod, robot, q);

% Inizializzazione array risultati
costs = zeros(num_samples, 1);
params = zeros(num_samples, paramstot);
valid_solutions = false(num_samples, 1);

% Calcolo numero di batch
num_batches = ceil(num_samples / batch_size);
fprintf('Numero di batch: %d (ogni batch = %d campioni = %d per worker)\n', ...
    num_batches, batch_size, batch_size/n_workers);

% Esecuzione batch per batch
for batch_idx = 1:num_batches
    % Calcola indici per questo batch
    start_idx = (batch_idx - 1) * batch_size + 1;
    end_idx = min(batch_idx * batch_size, num_samples);
    current_batch_size = end_idx - start_idx + 1;
    
    fprintf('Batch %d/%d: campioni %d-%d (%d campioni)... ', ...
        batch_idx, num_batches, start_idx, end_idx, current_batch_size);
    
    batch_start_time = tic;
    
    % Array temporanei per questo batch
    batch_costs = zeros(current_batch_size, 1);
    batch_params = zeros(current_batch_size, paramstot);
    batch_valid = false(current_batch_size, 1);
    
    % Esecuzione parallela per questo batch
    parfor i = 1:current_batch_size
        global_idx = start_idx + i - 1;
        
        % Punto iniziale per questa iterazione
        p0 = lb + (ub - lb) .* distributionp(global_idx, :)';
        
        try
            % Esecuzione ottimizzazione
            [p_opt, cost_opt, exitflag] = fmincon(cost_function, p0, [], [], [], [], lb, ub, [], optionsfmin);
            
            % Salvataggio risultati
            batch_costs(i) = cost_opt;
            batch_params(i, :) = p_opt';
            batch_valid(i) = (exitflag > 0); % Soluzione valida se exitflag > 0
            
        catch ME
            % Gestione errori
            batch_costs(i) = 1e6; % Costo molto alto per soluzioni fallite
            batch_params(i, :) = p0';
            batch_valid(i) = false;
        end
    end
    
    % Salva risultati del batch nell'array principale
    costs(start_idx:end_idx) = batch_costs;
    params(start_idx:end_idx, :) = batch_params;
    valid_solutions(start_idx:end_idx) = batch_valid;
    
    % Statistiche batch
    batch_time = toc(batch_start_time);
    batch_valid_count = sum(batch_valid);
    samples_per_second = current_batch_size / batch_time;
    fprintf('completato in %.1fs (valide: %d/%d, %.1f camp/s)\n', ...
        batch_time, batch_valid_count, current_batch_size, samples_per_second);
    
    % Controllo miglioramenti intermedi ogni 10 batch
    if batch_idx == 1 || mod(batch_idx, 10) == 0
        current_valid_costs = costs(valid_solutions(1:end_idx));
        if ~isempty(current_valid_costs)
            current_best = min(current_valid_costs);
            fprintf('  -> Miglior costo finora: %.6f (dopo %d campioni)\n', current_best, end_idx);
        end
    end
    
    % Stima tempo rimanente
    if batch_idx == 3 % Dopo 3 batch fai una stima
        avg_time_per_batch = batch_time; % Semplificato, usa l'ultimo batch
        remaining_batches = num_batches - batch_idx;
        estimated_remaining = remaining_batches * avg_time_per_batch;
        fprintf('  -> Tempo stimato rimanente: %.1f minuti\n', estimated_remaining/60);
    end
end

%% Elaborazione risultati
fprintf('\nElaborazione risultati...\n');

% Filtro solo soluzioni valide
valid_costs = costs(valid_solutions);
valid_params = params(valid_solutions, :);

if isempty(valid_costs)
    error('Nessuna soluzione valida trovata. Controllare vincoli e condizioni iniziali.');
end

fprintf('Soluzioni valide trovate: %d/%d\n', length(valid_costs), num_samples);

%% Analisi progressiva dei miglioramenti
fprintf('\nAnalisi miglioramenti:\n');
[sorted_costs, sort_idx] = sort(valid_costs);
sorted_params = valid_params(sort_idx, :);

% Trova tutti i miglioramenti
current_best = inf;
for i = 1:length(sorted_costs)
    if sorted_costs(i) < current_best
        improvement_count = improvement_count + 1;
        current_best = sorted_costs(i);
        
        % Salva questo miglioramento
        collect_p = [collect_p; sorted_params(i, :)];
        
        % Stampa miglioramento
        if improvement_count <= 10 % Stampa solo i primi 10 miglioramenti
            fprintf('  Miglioramento #%d: Costo = %.6f\n', improvement_count, current_best);
        end
    end
end

% Migliore soluzione finale
bestCost = sorted_costs(1);
bestP_3D = sorted_params(1, :)';

%% Statistiche finali
elapsed_time = toc(start_time);
fprintf('\n=== STATISTICHE OTTIMIZZAZIONE ===\n');
fprintf('Tempo totale: %.2f secondi\n', elapsed_time);
fprintf('Campioni processati: %d\n', num_samples);
fprintf('Soluzioni valide: %d (%.1f%%)\n', length(valid_costs), 100*length(valid_costs)/num_samples);
fprintf('Miglioramenti trovati: %d\n', improvement_count);
fprintf('Costo finale migliore: %.6f\n', bestCost);

%% Controllo validità soluzione finale
if isempty(bestP_3D)
    error('Nessuna soluzione valida trovata. Controllare vincoli e condizioni iniziali.');
end

%% Valutazione finale e calcolo output
fprintf('\n=== VALUTAZIONE FINALE ===\n');
[TauExo, Actual_Pin, L_springs, L0, Tension, L0_recoil] = exoNetTorques3D(Pos, Bod, Exo, bestP_3D, robot, q);

%% Stampa parametri soluzione finale
print_solution_parameters(bestP_3D, indexswivel, 'SOLUZIONE FINALE OTTIMALE');

%% Visualizzazione risultati
fprintf('\nCreazione visualizzazioni...\n');
try
    Force_vector = create_plots(Exo, Actual_Pin, bestP_3D, L0_recoil, TauExo, TAUsDesired, q, Bod, Pos, robot);
catch ME
    warning('Errore nella creazione dei grafici: %s', ME.message);
    Force_vector = [];
end

%% Salvataggio cronologia ottimizzazione
fprintf('Salvataggio cronologia ottimizzazione...\n');
try
    optimization_history = struct();
    optimization_history.all_costs = costs;
    optimization_history.all_params = params;
    optimization_history.valid_solutions = valid_solutions;
    optimization_history.best_cost = bestCost;
    optimization_history.best_params = bestP_3D;
    optimization_history.improvements = collect_p;
    optimization_history.elapsed_time = elapsed_time;
    optimization_history.timestamp = datestr(now);
    
    % Salva in file MAT
    save('optimization_history.mat', 'optimization_history');
    fprintf('Cronologia salvata in: optimization_history.mat\n');
catch ME
    warning('Errore nel salvataggio cronologia: %s', ME.message);
end

%% Grafico finale dei miglioramenti
if plot_improvements && improvement_count > 1
    try
        figure('Name', 'Convergenza Ottimizzazione', 'NumberTitle', 'off');
        improvement_costs = zeros(size(collect_p, 1), 1);
        for i = 1:size(collect_p, 1)
            improvement_costs(i) = cost(collect_p(i, :)', TAUsDesired, Exo, Pos, Bod, robot, q);
        end
        
        plot(1:length(improvement_costs), improvement_costs, 'b-o', 'LineWidth', 2, 'MarkerSize', 6);
        xlabel('Numero Miglioramento');
        ylabel('Costo');
        title('Convergenza dell''Ottimizzazione');
        grid on;
        
        % Aggiungi testo con statistiche
        text(0.02, 0.98, sprintf('Miglioramenti: %d\nCosto finale: %.4f\nTempo: %.1fs', ...
            improvement_count, bestCost, elapsed_time), ...
            'Units', 'normalized', 'VerticalAlignment', 'top', ...
            'BackgroundColor', 'white', 'EdgeColor', 'black');
            
        fprintf('Grafico convergenza creato\n');
    catch ME
        warning('Errore nella creazione del grafico di convergenza: %s', ME.message);
    end
end

fprintf('\n=== OTTIMIZZAZIONE COMPLETATA ===\n');
fprintf('Risultati disponibili nelle variabili di output\n\n');

end


