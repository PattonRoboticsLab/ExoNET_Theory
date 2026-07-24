function compare_partha(p, Bod, Pos_sw, Exo, robot, T)

[p, c, TAUs, ~, F] = loadBestPartha();

%% Le tue torque già calcolate
TauElevation     = TAUs(:, 1);
TauElbow_flexion = TAUs(:, 2);

size(TauElevation)

Tau_elevation = TauElevation(1:2:end,:);
Tau_abduction = TauElevation(1:2:end,:);
Tau_swivel_init = TauElevation(2:2:end,:);

% Change Tau_swivel
factor = pi/180*linspace(0,90,10);
for i = 1:length(Tau_swivel_init)
    Tau_swivel_init(i,:) = Tau_swivel_init(9,:) * cos(factor(i));
end

for i = 1:length(Tau_swivel_init)
    Tau_abduction(i,:) = Tau_abduction(i,:) * cos(factor(i))^5;
end

%% Plot the torques
%% Angoli per gli assi X (coerenti con le lunghezze dei vettori)
ang_elev = linspace( 0, 100, numel(T.Tau_desired_sh_elevation));
ang_abd  = linspace( 0, 100, numel(T.Tau_desired_sh_abduction));
ang_sw   = linspace(  0, 90, numel(T.Tau_desired_sh_swivel));

%% Vettori Y (colonne, così non ci sono ambiguità)
y_elev_des = T.Tau_desired_sh_elevation(:);
y_elev_exo = T.Tau_exo_sh_elevation(:);
y_elev_base = Tau_elevation(:);        % verde (senza specificità)

y_abd_des = T.Tau_desired_sh_abduction(:);
y_abd_exo = T.Tau_exo_sh_abduction(:);
y_abd_base = Tau_abduction(:);         % verde (senza specificità)

y_sw_des = T.Tau_desired_sh_swivel(:);
y_sw_exo = T.Tau_exo_sh_swivel(:);
y_sw_base = Tau_swivel_init(:);        % verde (senza specificità)

%% (Opzionale) Allineamento di lunghezze, nel caso qualcosa non combaci
nE = min([numel(ang_elev), numel(y_elev_des), numel(y_elev_exo), numel(y_elev_base)]);

ang_elev = ang_elev(1:nE);  y_elev_des = y_elev_des(1:nE);  y_elev_exo = y_elev_exo(1:nE);  y_elev_base = y_elev_base(1:nE);

nA = min([numel(ang_abd), numel(y_abd_des), numel(y_abd_exo), numel(y_abd_base)]);
ang_abd = ang_abd(1:nA);  y_abd_des = y_abd_des(1:nA);  y_abd_exo = y_abd_exo(1:nA);  y_abd_base = y_abd_base(1:nA);

nS = min([numel(ang_sw), numel(y_sw_des), numel(y_sw_exo), numel(y_sw_base)]);
ang_sw = ang_sw(1:nS);  y_sw_des = y_sw_des(1:nS);  y_sw_exo = y_sw_exo(1:nS);  y_sw_base = y_sw_base(1:nS);

%% --- Grafico 1: Elevation ---
figure('Name','Shoulder Elevation Torque'); 
plot(ang_elev, y_elev_des, 'r-', 'LineWidth', 2); hold on;
plot(ang_elev, y_elev_base,'g-', 'LineWidth', 2);
grid on; xlabel('Elevation angle (deg)'); ylabel('Elevation Torque (N·m)');
title('Shoulder Elevation in sagital plane'); legend({'Desired','ExoNET'}, 'Location','best');

%% --- Grafico 2: Abduction ---
figure('Name','Shoulder Abduction Torque'); 
plot(ang_abd, y_abd_des, 'r-', 'LineWidth', 2); hold on;
plot(ang_abd, y_abd_base,'g-', 'LineWidth', 2);
grid on; xlabel('Abduction angle (deg)'); ylabel('Abduction Torque (N·m)');
title('Shoulder abduction'); legend({'Desired','ExoNET'}, 'Location','best');

%% --- Grafico 3: Swivel ---
figure('Name','Shoulder Swivel Torque'); 
plot(ang_sw, y_sw_des, 'r-', 'LineWidth', 2); hold on;
plot(ang_sw, y_sw_base,'g-', 'LineWidth', 2);
grid on; xlabel('Swivel angle (deg)'); ylabel('Abduction Torque (N·m)');
title('Shoulder Swivel in position'); legend({'Desired','ExoNET'}, 'Location','best');

%% --- Grafico 1: Elevation ---
figure('Name','Shoulder Elevation Torque'); 
plot(ang_elev, y_elev_des, 'r-', 'LineWidth', 2); hold on;
plot(ang_elev, y_elev_exo, 'b-', 'LineWidth', 2);
plot(ang_elev, y_elev_base,'g-', 'LineWidth', 2);
grid on; xlabel('Elevation angle (deg)'); ylabel('Elevation Torque (N·m)');
title('Shoulder Elevation in sagital plane'); legend({'Desired','3D ExoNET','ExoNET'}, 'Location','best');

%% --- Grafico 2: Abduction ---
figure('Name','Shoulder Abduction Torque'); 
plot(ang_abd, y_abd_des, 'r-', 'LineWidth', 2); hold on;
plot(ang_abd, y_abd_exo, 'b-', 'LineWidth', 2);
plot(ang_abd, y_abd_base,'g-', 'LineWidth', 2);
grid on; xlabel('Abduction angle (deg)'); ylabel('Abduction Torque (N·m)');
title('Shoulder abduction'); legend({'Desired','3D ExoNET','ExoNET'}, 'Location','best');

%% --- Grafico 3: Swivel ---
figure('Name','Shoulder Swivel Torque'); 
plot(ang_sw, y_sw_des, 'r-', 'LineWidth', 2); hold on;
plot(ang_sw, y_sw_exo, 'b-', 'LineWidth', 2);
plot(ang_sw, y_sw_base,'g-', 'LineWidth', 2);
grid on; xlabel('Swivel angle (deg)'); ylabel('Abduction Torque (N·m)');
title('Shoulder Swivel in position'); legend({'Desired','3D ExoNET','ExoNET'}, 'Location','best');

%% Setup angoli
nAngles = 10;
phi1 = pi/180*linspace( -80+10^-6, 10+10^-6, nAngles);  % elevation spalla
phi2 = pi/180*linspace(   10+10^-6, 45+10^-6, 2);  
PHIs=[];  
for i=1:length(phi1)          % nested 2-loop establishes grid of phi's
  for j=1:length(phi2), PHIs = [PHIs; phi1(i),phi2(j)]; end % stack up list
end 
% FIX: Update nAngles to reflect actual number of combinations
nAngles = size(PHIs, 1);  % This will be 20 (10 × 2)

% Lunghezze segmenti
L1 = Bod.L(1);  % upper arm
L2 = Bod.L(2);  % forearm

phi3 = zeros(size(PHIs(:,1),1),1);  % plane angle (abduction) = 0 per piano sagittale

%% Posizioni con la TUA cinematica
Pos_sw.sh = [0.1855, -0.00435, 1.35];  % spalla (come nel tuo codice) [1x3]

phi12 = PHIs(:,1) + PHIs(:,2);  % Sum phi's

% Converti phi1, phi2, phi3 in colonne se necessario
phi1 = PHIs(:,1);
phi2 = PHIs(:,2);
phi3 = phi3(:);
phi12 = phi12(:);

% Position of the elbow (upper arm endpoint)
Pos.elbowSwivel = repmat(Pos_sw.sh, nAngles, 1) + L1 * [ ...
    cos(phi3).*cos(phi1), ...
    sin(phi3).*cos(phi1), ...
    sin(phi1) ];

Pos.el2wr = L2 * [ ...
    cos(phi3).*cos(phi12), ...
    sin(phi3).*cos(phi12), ...
    sin(phi12) ];

Pos.wrSwivel = Pos.elbowSwivel + Pos.el2wr;

% Calcola q dalla cinematica inversa usando posizioni RELATIVE
q = zeros(nAngles, 4);
for i = 1:nAngles
    % Usa posizioni relative alla spalla del robot (origine)
    shoulder_robot = [0, 0, 0];
    elbow_relative = Pos.elbowSwivel(i,:) - Pos_sw.sh;
    wrist_relative = Pos.wrSwivel(i,:) - Pos_sw.sh;

    q(i,:) = computeJointAngles3D(shoulder_robot, elbow_relative, wrist_relative);
end

%% Costruisci TauSh e TauEl
TauSh = zeros(nAngles, 3);
TauSh(:, 1) = 0;
TauSh(:, 2) = TauElevation;
TauSh(:, 3) = 0;

TauEl = zeros(nAngles, 3);
TauEl(:, 1) = TauElbow_flexion;
TauEl(:, 2) = 0;
TauEl(:, 3) = 0;

% %% Chiama plotVectField3D_opt (restituisce solo 4 righe)
% Force_vector_initial = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TauSh, TauEl, ...
%     [0 0 1], 2, 0.5, 1, ...
%     'TorquesAreJoint', true, ...
%     'AssumeZeroMoment', true, ...
%     'TauAreActuator', true, ...
%     'BodyName', 'hand', ...
%     'NoPlot', true, ...
%     'ForceApplicationPoint', 'wrist', ...
%     'Stride', 1);
% 
% %% Setup per il loop
% phi3 = pi/180*linspace(0+10^-6, 90+10^-6, 10); % Phi3 plane of elevation
%     factor = [0.1, 0.1, 1, 1, 1];
% 
% % Inizializza Force_vector con la dimensione corretta
% % j va da 2 a 4 (3 valori), i va da 0 a 3 (4 valori) -> 3*4 = 12 righe
% Force_vect_ext = [];    Force_vect_flx = [];
% 
% %% Adjust the vector F
% for j = 1:5
%     for i = 1:5
% 
%         Force_vect_flx = [Force_vect_flx;   ...
%             Force_vector_initial(i,1) * cos(phi3(j)+pi/2) * factor(j), ...
%             Force_vector_initial(i,1) * sin(phi3(j)+pi/2) * factor(j), ...
%             Force_vector_initial(i,1) * factor(j)];
% 
%         Force_vect_ext = [Force_vect_ext; ...
%             Force_vector_initial(i,1) * cos(phi3(j)) * factor(j), ...
%             Force_vector_initial(i,1) * sin(phi3(j)) * factor(j), ...
%             Force_vector_initial(i,1) * factor(j)];
% 
%     end
% end
% 
% % Ora Force_vector, F_real_des, F_real_flexed hanno tutti 12 righe
% 
% %% Verifica dimensioni prima di plottare
% fprintf('Force_vect_ext size: [%d x %d]\n', size(Force_vect_ext));
% %fprintf('F_real_flexed size: [%d x %d]\n', size(F_real_flexed));
% fprintf('desFlx size: [%d x %d]\n', size(desFlx));
% fprintf('desExt size: [%d x %d]\n', size(desExt));

% Assicurati che p0_flex e p0_ext abbiano 12 righe
% Se non le hanno, devi adattarli

%% Plot
% aScalexo = 0.002;  aScale = 1; 
% Force_vect_ext = Force_vect_ext * aScalexo;
% Force_vect_flx = Force_vect_flx * aScalexo;
% 
% figure; nancy_body_normpose; hold on; 
% F_exo = showVectField([p0_ext(:,1), p0_ext(:,2), p0_ext(:,3)],    [Force_vect_ext(:,1), Force_vect_ext(:,2), Force_vect_ext(:,3)], aScale, 'b');
%  hef = showVectField([p0_flex(:,1), p0_flex(:,2), p0_flex(:,3)], [Force_vect_flx(:,1), Force_vect_flx(:,2), Force_vect_flx(:,3)],  aScale, 'b');
% 
%   hdf = showVectField([p0_flex(:,1), p0_flex(:,2), p0_flex(:,3)], [desFlx(:,1), desFlx(:,2), desFlx(:,3)], aScale, 'r');
% F_des = showVectField([p0_ext(:,1), p0_ext(:,2), p0_ext(:,3)],  [desExt(:,1), desExt(:,2), desExt(:,3)], aScale, 'r');
% 
% xlabel('Frontal plane'); ylabel('Sagital plane'); zlabel('Trasversal plane');
% legend( [hdf hef], {'Desired','Exo\_opt'}, 'Location','best');
% axis equal; set(gca, 'XTick', [], 'YTick', [], 'ZTick', []); zlim([0.6 1.9]); hold off;
% 
% % to have figure with normalpose and vector field
% % figure; nancy_body_normpose; hold on; 
% % F_des = showVectField([p0_ext(:,1), p0_ext(:,2), p0_ext(:,3)],  [desExt(:,1), desExt(:,2), desExt(:,3)], aScale, 'r');
% % xlabel('Frontal plane'); ylabel('Sagital plane'); zlabel('Trasversal plane');
% % axis equal; set(gca, 'XTick', [], 'YTick', [], 'ZTick', []); zlim([0.6 1.9]); hold off;
% Force_vector = struct(); % Oppure Force_vector.qualcosa = ...
% Force_vector.desired_extended = desExt;
% Force_vector.exo_extended = Force_vect_ext;
% Force_vector.desired_flexed = desFlx;
% Force_vector.exo_flexed = Force_vect_flx;
% 
% out = compareVectors(Force_vector, 'Scale', 0.6, 'Normalize', true, ...
%     'OriginExtended',[0.192784,0.390403,1.20328], 'OriginFlexed',[0.311632,0.317726,1.28505]);
% 
% figure; nancy_body; hold on;  
% %err_all = [TauResidual.sh_firstplane; TauResidual.sh_secondplane];
% plotErrorVoxelGrid(Pos, Exo, err_all, 'which', 'wrist', 'nx', 14, 'ny', 12, 'nz', 10, ...
%     'alphaMin', 0.1, ...
%     'alphaMax', 2, ...
%     'alphaScale', 'sqrt', ...
%     'alphaRef', 0.1, ...
%     'titleStr', 'Residual error', ...
%     'shiftVec', [-0.2, 0, 0]);

end