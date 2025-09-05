function [TauResidual, RMSE_exo] = Residualtorque(TauExo, TAUsDesired, Pos, Bod, Exo, Tension, bestCost, bestP  )

%% Data to plot the bars
rotations  = linspace(0 + 10^-6,  135,         Bod.nAngles_z); % Phi3 rotation around z axis
elevations = linspace(10 + 10^-6, 135 + 10^-6, Bod.nAngles);   % Phi1 elevation angle

[rotations_grid, elevations_grid] = meshgrid(rotations, elevations);
rotations_vector = rotations_grid(:);   elevations_vector = elevations_grid(:);

%% Check how many constraint elements there are
nparams = Exo.nParamsSh * round(Exo.numbconstraints(2)); 
p = zeros(nparams+1, 4);          
p(:,1) = bestP( 1 : nparams+1 );
p(:,2) = bestP( nparams+2 : nparams*2+2 );

numbsh = round( p(1,1) );   numbswivel = round( p(1,2) ); 

%% Plot the residual torque for the shoulder for first plane of elevation (0°)
Tau_desired_sh_firstplane = vecnorm( TAUsDesired.TauSh_tot(1:2:end,:), 2, 2 );
    Tau_exo_sh_firstplane = vecnorm( TauExo.elevationSh(1:2:end,:), 2, 2 );

TauResidual.sh_firstplane = Tau_desired_sh_firstplane - Tau_exo_sh_firstplane;

error_sh_firstplane = reshape( TauResidual.sh_firstplane, [], Bod.nAngles_z );

% Convert error_sh to a vector for scatter
error_sh_vector_firstplane = error_sh_firstplane(:);

% Normalize error_sh_vector for color mapping
normalized_error_sh = error_sh_vector_firstplane / max(error_sh_vector_firstplane);

%% Plot the residual torque for the shoulder for second plane of elevation (90°)
Tau_desired_sh_secondplane = vecnorm( TAUsDesired.TauSh_tot(2:2:end,:), 2, 2 );
    Tau_exo_sh_secondplane = vecnorm( TauExo.elevationSh(2:2:end,:), 2, 2 );

TauResidual.sh_secondplane = Tau_desired_sh_secondplane - Tau_exo_sh_secondplane;

error_sh_secondplane = reshape( TauResidual.sh_secondplane, [], Bod.nAngles_z );

% Convert error_sh to a vector for scatter
error_sh_vector_secondplane = error_sh_secondplane(:);

% Normalize error_sh_vector for color mapping
normalized_error_sh_secondplane = error_sh_vector_secondplane / max(error_sh_vector_secondplane);

%% Define limits for the color
min_error = min([min(error_sh_vector_firstplane), min(error_sh_vector_secondplane)]);
max_error = max([max(error_sh_vector_firstplane), max(error_sh_vector_secondplane)]);

%% Plot for the first plane of flexion and rotation (0°)
n = 256;  half_n = floor(n/2);
blue_to_white = [linspace(0, 1, half_n)', linspace(0, 1, half_n)', ones(half_n, 1)]; % From blue to white
white_to_red  = [ones(half_n, 1), linspace(1, 0, half_n)', linspace(1, 0, half_n)']; % From white to red
custom_colormap = [blue_to_white; white_to_red];

figure('Name','Error for shoulder'); hold on;
title('0° internal rotation / 0° elbow flexion');

dRot = mean(diff(rotations));     dElev = mean(diff(elevations));   

 rot_edges = [rotations - dRot/2, rotations(end) + dRot/2];
elev_edges = [elevations - dElev/2, elevations(end) + dElev/2];

[rot_edge_grid, elev_edge_grid] = meshgrid(rot_edges, elev_edges);

Z = error_sh_firstplane;

Z_padded = nan(size(Z) + 1); Z_padded(1:end-1, 1:end-1) = Z;

h = surf(rot_edge_grid, elev_edge_grid, zeros(size(Z_padded)), Z_padded, 'EdgeColor', 'k');
view(2); colormap(custom_colormap);


% Colorbar
c_sh = colorbar;    c_sh.Label.String = 'Error torque Nm';
c_sh.Label.FontSize = 12;  c_sh.Label.FontWeight = 'bold';

cmax = max(abs([min_error, max_error])); % Maximum absolute error
clim([-cmax, cmax]);

xticks(rotations);   yticks(elevations);
xlabel('Plane of elevation');                 ylabel('Elevation');
xticks(rotations);                            yticks(elevations);
xticklabels(cellfun(@(x) sprintf('%.1f°', x), num2cell(rotations), 'UniformOutput', false));  
yticklabels(cellfun(@(x) sprintf('%.1f°', x), num2cell(elevations), 'UniformOutput', false));
xlim([rot_edges(1), rot_edges(end)]);         ylim([elev_edges(1), elev_edges(end)]);

hold off;

%% Plot for the second plane of flexion and rotation (90°)
figure('Name','Error for shoulder'); hold on;
title('90° internal rotation / 90° elbow flexion');

Z = error_sh_secondplane;
Z_padded = nan(size(Z) + 1);  Z_padded(1:end-1, 1:end-1) = Z;

h = surf(rot_edge_grid, elev_edge_grid, zeros(size(Z_padded)), Z_padded, 'EdgeColor', 'k');
view(2); colormap(custom_colormap);

c_sh_second = colorbar;            c_sh_second.Label.String = 'Error torque Nm';
c_sh_second.Label.FontSize = 12;   c_sh_second.Label.FontWeight = 'bold';
clim([-cmax, cmax])

xlabel('Plane of elevation');                 ylabel('Elevation');
xticks(rotations);                            yticks(elevations);
xticklabels(cellfun(@(x) sprintf('%.1f°', x), num2cell(rotations),  'UniformOutput', false));  
yticklabels(cellfun(@(x) sprintf('%.1f°', x), num2cell(elevations), 'UniformOutput', false));
xlim([rot_edges(1), rot_edges(end)]);         ylim([elev_edges(1), elev_edges(end)]);
hold off;

%% Evaluate the RMSE
n_points = length( Pos.elbowSwivel );
RMSE_exo = sqrt( (1/n_points) * bestCost );

end