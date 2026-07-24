function Residualtorque(Bod, Exo, bestP )
% 
[Pos_el, Bod_el, Exo_el] = SetUp_ElAbd();
[ TAUsDesired_elabd, MaxTorques_elabd, robot_elabd, q_elabd ] = weightEffect3D( Bod_el, Pos_el, Exo_el);
[TauExo_elabd, Actual_Pin, L_springs, L0, Tension, func_tension] = exoNetTorques3D(Pos_el, Bod_el, Exo_el, bestP);

[Pos_sw, Bod_sw, Exo_sw] = SetUp_Swivel();
[ TAUsDesired_sw, MaxTorques_sw, robot_sw, q_sw ] = weightEffect3D( Bod_sw, Pos_sw, Exo_sw);
[TauExo_sw, Actual_Pin, L_springs, L0, Tension, func_tension] = exoNetTorques3D(Pos_sw, Bod_sw, Exo_sw, bestP);

%% Check how many constraint elements there are
nparams = Exo_el.nParamsSh * round(Exo_el.numbconstraints(2)); % take the number of parameter from Exo
p = zeros(nparams+1, 4);          
p(:,1) = bestP( 1 : nparams+1 );
p(:,2) = bestP( nparams+2 : nparams*2+2 );

numbsh = round( p(1,1) );   numbswivel = round( p(1,2) ); 

%% Extrapolate the 3 torques for the three selected movements
T.Tau_desired_sh_elevation = vecnorm( TAUsDesired_elabd.TauSh_tot(1:2:20,:), 2, 2 );
T.Tau_desired_sh_abduction = vecnorm( TAUsDesired_elabd.TauSh_tot(21:2:end,:), 2, 2 );

T.Tau_exo_sh_elevation = vecnorm( TauExo_elabd.elevationSh(1:2:20,:), 2, 2 );
T.Tau_exo_sh_abduction = vecnorm( TauExo_elabd.elevationSh(21:2:end,:), 2, 2 );

T.Tau_desired_sh_swivel = vecnorm( TAUsDesired_sw.TauSh_tot(1:end,:), 2, 2 );

T.Tau_exo_sh_swivel = vecnorm( TauExo_sw.elevationSh(1:end,:), 2, 2 );

%% Analyze Partha's
compare_partha(p, Bod, Pos_el, Exo, robot_sw, T);

% %% Plot the vector field
% scale_factor_ext = 0.02;
%     scale_factor = 0.04;
% scale_factor_err = 0.5;
% 
% desExt = Force_vector.desired_extended * scale_factor_ext;
% exoExt = Force_vector.exo_extended * scale_factor_ext;
% 
% desFlx = Force_vector.desired_flexed * scale_factor;
% exoFlx = Force_vector.exo_flexed * scale_factor;  
% 
% errExt = ( desExt - exoExt ) * scale_factor_err;
% errFlx = ( desFlx - exoFlx ) * scale_factor_err;
% 
% %% correct if some vector is too big
% threshold = 0.5;   
% V = {desExt, exoExt, desFlx, exoFlx, errExt, errFlx};
% names = {'desExt','exoExt','desFlx','exoFlx','errExt','errFlx'};
% 
% for i = 1:numel(V)
%     n = norm(V{i}(:));
%     if n > threshold
%         V{i} = V{i} * (threshold / n);
%     end
% end
% [desExt, exoExt, desFlx, exoFlx, errExt, errFlx] = deal(V{:});

%% Data to plot the bars
% rotations  = linspace( 0 + 10^-6,  135,         Bod.nAngles_z); % Phi3 rotation around z axis
% elevations = linspace( 10 + 10^-6, 135 + 10^-6, Bod.nAngles);   % Phi1 elevation angle
% 
% [rotations_grid, elevations_grid] = meshgrid(rotations, elevations);
% rotations_vector = rotations_grid(:);     elevations_vector = elevations_grid(:);

%% Plot for the first plane of flexion and rotation (0°)
% n = 256;  half_n = floor(n/2);
% blue_to_white = [linspace(0, 1, half_n)', linspace(0, 1, half_n)', ones(half_n, 1)]; % From blue to white
% white_to_red  = [ones(half_n, 1), linspace(1, 0, half_n)', linspace(1, 0, half_n)']; % From white to red
% custom_colormap = [blue_to_white; white_to_red];
% 
% figure('Name','Error for shoulder'); hold on;
% title('0° internal rotation / 0° elbow flexion');
% 
% dRot = mean(diff(rotations));     dElev = mean(diff(elevations));   
% 
%  rot_edges = [rotations - dRot/2, rotations(end) + dRot/2];
% elev_edges = [elevations - dElev/2, elevations(end) + dElev/2];
% 
% [rot_edge_grid, elev_edge_grid] = meshgrid(rot_edges, elev_edges);
% 
% Z = error_sh_firstplane;
% 
% Z_padded = nan(size(Z) + 1); Z_padded(1:end-1, 1:end-1) = Z;
% 
% h = surf(rot_edge_grid, elev_edge_grid, zeros(size(Z_padded)), Z_padded, 'EdgeColor', 'k');
% view(2); colormap(custom_colormap);
% 
% % Colorbar
% c_sh = colorbar;    c_sh.Label.String = 'Error torque Nm';
% c_sh.Label.FontSize = 12;  c_sh.Label.FontWeight = 'bold';
% 
% cmax = max(abs([min_error, max_error])); % Maximum absolute error
% clim([-cmax, cmax]);
% 
% xticks(rotations);   yticks(elevations);
% xlabel('Plane of elevation');                 ylabel('Elevation');
% xticks(rotations);                            yticks(elevations);
% xticklabels(cellfun(@(x) sprintf('%.1f°', x), num2cell(rotations), 'UniformOutput', false));  
% yticklabels(cellfun(@(x) sprintf('%.1f°', x), num2cell(elevations), 'UniformOutput', false));
% xlim([rot_edges(1), rot_edges(end)]);         ylim([elev_edges(1), elev_edges(end)]);
% 
% hold off;

%% Plot for the second plane of flexion and rotation (90°)
% figure('Name','Error for shoulder'); hold on;
% title('90° internal rotation / 90° elbow flexion');
% 
% Z = error_sh_secondplane;
% Z_padded = nan(size(Z) + 1);  Z_padded(1:end-1, 1:end-1) = Z;
% 
% h = surf(rot_edge_grid, elev_edge_grid, zeros(size(Z_padded)), Z_padded, 'EdgeColor', 'k');
% view(2); colormap(custom_colormap);
% 
% c_sh_second = colorbar;            c_sh_second.Label.String = 'Error torque Nm';
% c_sh_second.Label.FontSize = 12;   c_sh_second.Label.FontWeight = 'bold';
% clim([-cmax, cmax])
% 
% xlabel('Plane of elevation');                 ylabel('Elevation');
% xticks(rotations);                            yticks(elevations);
% xticklabels(cellfun(@(x) sprintf('%.1f°', x), num2cell(rotations),  'UniformOutput', false));  
% yticklabels(cellfun(@(x) sprintf('%.1f°', x), num2cell(elevations), 'UniformOutput', false));
% xlim([rot_edges(1), rot_edges(end)]);         ylim([elev_edges(1), elev_edges(end)]);
% hold off;
% 
% %% Plot in the space the error with voxels
% figure; nancy_body; hold on;  
% err_all = [TauResidual.sh_firstplane; TauResidual.sh_secondplane];
% plotErrorVoxelGrid(Pos, Exo, err_all, 'which', 'wrist', 'nx', 14, 'ny', 12, 'nz', 10, ...
%     'alphaMin', 0.1, ...
%     'alphaMax', 2, ...
%     'alphaScale', 'sqrt', ...
%     'alphaRef', 0.1, ...
%     'titleStr', 'Residual error', ...
%     'shiftVec', [-0.2, 0, 0]);

%% Compare the vectors
% out = compareVectors(Force_vector, 'Scale', 0.6, 'Normalize', true, ...
%     'OriginExtended',[0.192784,0.390403,1.20328], 'OriginFlexed',[0.311632,0.317726,1.28505]);

%% Figure for just the force field
% aScale=1;
% figure('Name','Force field'); nancy_body; hold on; %title('0° internal rotation / 0° elbow flexion');
% hdd = showVectField([p0_ext(:,1),p0_ext(:,2),p0_ext(:,3)],    [desExt(:,1), desExt(:,2), desExt(:,3)], aScale, 'r');
% hdf = showVectField([p0_flex(:,1),p0_flex(:,2),p0_flex(:,3)], [desFlx(:,1), desFlx(:,2), desFlx(:,3)], aScale, 'r');
% legend( hdd, {'Desired'}, 'Location','best'); axis equal;  zlim([0.6 1.9]);

%% Figure for the elbow EXTENDED 
% figure('Name','Force field'); nancy_body; hold on; %title('0° internal rotation / 0° elbow flexion');
% hdd = showVectField([p0_ext(:,1),p0_ext(:,2),p0_ext(:,3)], [desExt(:,1), desExt(:,2), desExt(:,3)],aScale, 'r');
% hdf = showVectField([p0_flex(:,1),p0_flex(:,2),p0_flex(:,3)], [desFlx(:,1), desFlx(:,2), desFlx(:,3)],aScale, 'r');
% 
% hee = showVectField([p0_ext(:,1),p0_ext(:,2),p0_ext(:,3)], [exoExt(:,1), exoExt(:,2), exoExt(:,3)],aScale, 'b');
% hef = showVectField([p0_flex(:,1),p0_flex(:,2),p0_flex(:,3)], [exoFlx(:,1), exoFlx(:,2), exoFlx(:,3)],aScale, 'b');
% xlabel('Frontal plane'); ylabel('Sagital plane'); zlabel('Trasversal plane'); axis equal;
% legend( [hdd hee], {'Desired','Exo\_opt'}, 'Location','best'); axis equal;  zlim([0.6 1.9]); set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
% hold off;
% 
% figure('Name','Error field'); nancy_body; hold on; %title('0° internal rotation / 0° elbow flexion');
%  hError = showVectField([p0_ext(:,1),p0_ext(:,2),p0_ext(:,3)],   [errExt(:,1), errExt(:,2), errExt(:,3)],aScale, 'g');
% hErrorf = showVectField([p0_flex(:,1),p0_flex(:,2),p0_flex(:,3)],[errFlx(:,1), errFlx(:,2), errFlx(:,3)],aScale, 'g');
% xlabel('Frontal plane'); ylabel('Sagital plane'); zlabel('Trasversal plane'); axis equal;
% legend( hError, {'Error'}, 'Location','best'); axis equal; zlim([0.6 1.9]); set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
% hold off;
% 
% %% Figure with nancy body elbow extended
% figure('Name','Force field elbow extended'); nancy_body; hold on; title('0° internal rotation / 0° elbow flexion');
% %legend4 = PlotPin( Exo, Actual_Pin, shoulder_pin, shoulder_el, shoulder_sw, elbow_sw, elbow_el, bestP_3D, center_back, wrist);
% Force_vector.exo_extended = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TauExo.elevationSh, TauExo.elevationEl, ...
%                                 [0, 0, 1], 1.5,   0.7, 1, 'Transparency', 0.7);
% Force_vector.desired_extended = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TAUsDesired.TauSh_tot,  TAUsDesired.TauEl_tot, ...
%                                 [1, 0, 0], 1.5,   0.7, 1, 'IsDesired', true, 'Transparency', 0.9, 'Offset', [0.001, 0.001, 0.001]); hold off; % grid off; legend(legend4);
% xlabel('Frontal plane'); ylabel('Sagital plane'); zlabel('Trasversal plane'); set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
% legend( [hdf hef], {'Desired','Exo\_opt'}, 'Location','best');
% 
% %% Figure with nancy body elbow extended ERROR
% figure('Name','Pose elbow extended'); nancy_body; hold on; title('0° internal rotation / 0° elbow flexion');
% scatter3(p0_ext(:,1), p0_ext(:,2), p0_ext(:,3), ...
%     40, 'filled', 'r');   % 40 = dimensione marker, 'b' = blu
% axis equal;xlabel('Frontal plane'); ylabel('Sagital plane'); zlabel('Trasversal plane'); set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
% legend( [hdf hef], {'Desired','Exo\_opt'}, 'Location','best');
% 
% %% Figure with nancy body elbow flexed 
% figure('Name','Pose elbow flexed'); nancy_body; hold on; title('90° internal rotation / 90° elbow flexion');
% %legend5 = PlotPin_flex( Exo, Actual_Pin, shoulder_pin, shoulder_el_flex, shoulder_sw_flex, elbow_sw_flex, elbow_el_flex, bestP_3D, center_back, wrist_flex);
% scatter3(p0_flex(:,1), p0_flex(:,2), p0_flex(:,3), ...
%     40, 'filled', 'r');   % 40 = dimensione marker, 'b' = blu
% axis equal; xlabel('Frontal plane'); ylabel('Sagital plane'); zlabel('Trasversal plane'); set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
% legend( [hdf hef], {'Desired','Exo\_opt'}, 'Location','best');
% 
% %% Figure with nancy body elbow flexed ERROR
% figure('Name','Error field elbow extended'); nancy_body; hold on; title('0° internal rotation / 0° elbow flexion');
% %legend4 = PlotPin( Exo, Actual_Pin, shoulder_pin, shoulder_el, shoulder_sw, elbow_sw, elbow_el, bestP_3D, center_back, wrist);
% Force_vector.exo_extended = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TAUsDesired.TauSh_tot-TauExo.elevationSh, TAUsDesired.TauEl_tot-TauExo.elevationEl, ...
%                                 [0 1 0], 1.5,   0.7, 1, 'Transparency', 0.7);
% 



end