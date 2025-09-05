function recoilevaluation(Bod, Pos, Exo, pin, p, L_spring, L0, Tension)

nparams = Exo.nParamsSh * round(Exo.numbconstraints(2));    
indexswivel = nparams + 1;

%% Evaluate DL springs
for i = 1:round( p(1) )
    DL.swivel(:,i) = L_spring.swivel(:,i) - L0.swivel(i);
end
for i = 1:round( p( indexswivel+1 ) )
    DL.elevation(:,i) = L_spring.elevation(:,i) - L0.elevation(i);
end

maxDL.elevation = max(DL.elevation, [], 1) - min(DL.elevation, [], 1);
maxDL.swivel = max(DL.swivel, [], 1) - min(DL.swivel, [], 1);

%% Create the P vector for recoiling
L0_spring = [0, 1];  Kspring = [0, 1];
L_max = 1/2;         K_max = 1000;

Exo.P = [];  
for iteration = 1:round( p(1) ) + round( p( indexswivel+1 ) ) % For the swivel and elevation constraint
    Exo.P = [Exo.P; Kspring; L0_spring];
end

paramstot = length(Exo.P);  num_samples = 1;  
distributionp = lhsdesign(num_samples, paramstot); % Values from 0 to 1 uniformely distributed
ub = Exo.P(:, 2);           lb = Exo.P(:, 1);
%% Evaluation of the spring and attachment
L_rail = 0.3; offset = 0.1;
% Create the origin of the springs
for i = 1:round( bestP_3D(1) )
    Rail_origin_elevation(i,:) = pin.elevation - [0, L_rail, 0];
end
for i = 1:round( bestP_3D( indexswivel+1 ) )
    Rail_origin_swivel(i,:) = pin.swivel - [0, L_rail, 0];
end

Rail_ends_elevation = Rail_origin_elevation + [0, offset, 0];
Rail_ends_swivel = Rail_origin_swivel + [0, offset, 0];

%% Options for FMINCON
optionsfmin = optimoptions('fmincon', 'Display', 'iter', 'PlotFcn', [],...
     'OutputFcn', [], 'UseParallel', true,'Algorithm', 'interior-point', ...
     'MaxFunctionEvaluations', 20000, 'Diagnostics', 'on', 'FiniteDifferenceStepSize', 1e-6, ...
     'MaxIterations', 2000, 'OptimalityTolerance', 1e-6, 'FunValCheck', 'on',...
     'EnableFeasibilityMode', true, 'SubproblemAlgorithm', 'cg', 'ScaleProblem', true);

%% Loop multiple optimization
fprintf('Begin optimizations:\n');   
bestCost = 40000;    nTries = num_samples;    collect_p = [];

for TRY = 1:nTries; fprintf('\nOpt#%d of %d\n', TRY, nTries);        
    %% Fmincon
    % fprintf('\n... Using fmincon...')
    p0 = lb + (ub-lb) .* distributionp(TRY,:)';
    [p, new_cost] = fmincon(@(x) cost(x, TAUsDesired, Exo, Pos), p0, [], [], [], [], lb, ub, [], optionsfmin ); 
     
    %% Evaluation of the function
    if new_cost < bestCost   % If lower cost --> update new cost        
        fprintf(' Plot new solution: \n')
        fprintf(' c = %.3f ', new_cost ); 
        fprintf('\n p = \n');     
        fprintf(' %.3f  ', p( 1 : indexswivel)');               fprintf('\n');
        fprintf(' %.3f  ', p( indexswivel+1 : indexswivel*2)'); fprintf('\n');
        fprintf(' %.3f  ', p( indexswivel*2+1 : end)');         fprintf('\n');        
        bestCost = new_cost;    bestP_3D = p;  
        collect_p = [collect_p; bestP_3D];

    else
        fprintf(' Not an improvement, still best cost = %g\n', bestCost);
        fprintf(' Parameter were:\n');
        fprintf(' %.3f  ', p(1:indexswivel)');               fprintf('\n');
        fprintf(' %.3f  ', p(indexswivel+1:indexswivel*2)'); fprintf('\n');
        fprintf(' %.3f  ', p(indexswivel*2+1:end)');         fprintf('\n');
        if exist('bestP_3D', 'var')
            fprintf(' \nBest parameters are:\n');
            fprintf(' %.3f  ', bestP_3D(1:indexswivel)');               fprintf('\n');
            fprintf(' %.3f  ', bestP_3D(indexswivel+1:indexswivel*2)'); fprintf('\n');
            fprintf(' %.3f  ', bestP_3D(indexswivel*2+1:end)');         fprintf('\n');
        end
    end      
    if TRY == nTries
       break
    end   
end
