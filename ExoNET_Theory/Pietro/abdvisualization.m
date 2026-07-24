%% Visualizzazione SEMPLICE: solo 2 configurazioni (0° e 90°)
clear; clc; close all;

% ===== DATI DI ESEMPIO =====
Pos.sh = [0, 0, 0];              % Spalla
Pos.wr_init = [30, 20, -10];     % Polso (fisso)
Pos.el_init = [15, 10, -8];      % Gomito iniziale
Bod.gap = 3;                      % Distanza pin dal gomito

% Solo DUE angoli: 0° e 90°
Exo.theta = [0, -pi/2];

% ===== CALCOLO ROTAZIONE =====
axis_unit = (Pos.wr_init - Pos.sh) ./ norm(Pos.wr_init - Pos.sh);
sh_to_el = Pos.el_init - Pos.sh;
projection_length = dot(sh_to_el, axis_unit);
projection_point = Pos.sh + projection_length * axis_unit;
radial_vector = Pos.el_init - projection_point;
radial_distance = norm(radial_vector);
radial_unit = radial_vector ./ radial_distance;
tangent_unit = cross(axis_unit, radial_unit);
tangent_unit = tangent_unit ./ norm(tangent_unit);

% Inizializza
Pos.el_3D = zeros(2, 3);
Pos.wr_3D = zeros(2, 3);
Pos.gapel_3D = zeros(2, 3);
Pos.gapsw_3D = zeros(2, 3);

for j = 1:2
    % Polso fisso
    Pos.wr_3D(j,:) = Pos.wr_init;
    
    % Gomito ruota
    current_elbow = projection_point + ...
        radial_distance * (cos(Exo.theta(j)) * radial_unit + sin(Exo.theta(j)) * tangent_unit);
    Pos.el_3D(j,:) = current_elbow;
    
    % Sistema locale avambraccio
    forearm_axis = (Pos.wr_init - current_elbow) ./ norm(Pos.wr_init - current_elbow);
    elbow_to_shoulder = Pos.sh - current_elbow;
    projection_on_forearm = dot(elbow_to_shoulder, forearm_axis) * forearm_axis;
    in_plane_vector = elbow_to_shoulder - projection_on_forearm;
    in_plane_unit = in_plane_vector ./ norm(in_plane_vector);
    out_of_plane_unit = cross(forearm_axis, in_plane_unit);
    out_of_plane_unit = out_of_plane_unit ./ norm(out_of_plane_unit);
    
    % Pin solidali
    Pos.gapel_3D(j,:) = current_elbow + Bod.gap * in_plane_unit;
    Pos.gapsw_3D(j,:) = current_elbow + Bod.gap * out_of_plane_unit;
end

% ===== PLOT SEMPLICE =====
figure('Position', [100, 100, 1000, 800]);
hold on; grid on; axis equal;
title('Confronto: Configurazione INIZIALE (verde) vs FINALE (rosso)', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X [cm] - laterale'); 
ylabel('Y [cm] - anteriore'); 
zlabel('Z [cm] - superiore');

% SPALLA (nero)
plot3(Pos.sh(1), Pos.sh(2), Pos.sh(3), 'ko', 'MarkerSize', 20, 'MarkerFaceColor', 'k', 'LineWidth', 2);
text(Pos.sh(1)-2, Pos.sh(2)-2, Pos.sh(3)+3, 'SPALLA', 'FontSize', 12, 'FontWeight', 'bold');

% POLSO (blu)
plot3(Pos.wr_init(1), Pos.wr_init(2), Pos.wr_init(3), 'bo', 'MarkerSize', 20, 'MarkerFaceColor', 'b', 'LineWidth', 2);
text(Pos.wr_init(1)+2, Pos.wr_init(2), Pos.wr_init(3)+2, 'POLSO', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'b');

% ===== CONFIGURAZIONE 1: θ = 0° (VERDE) =====
% Braccio superiore
plot3([Pos.sh(1), Pos.el_3D(1,1)], [Pos.sh(2), Pos.el_3D(1,2)], [Pos.sh(3), Pos.el_3D(1,3)], ...
      'g-', 'LineWidth', 5);

% Avambraccio
plot3([Pos.el_3D(1,1), Pos.wr_3D(1,1)], [Pos.el_3D(1,2), Pos.wr_3D(1,2)], [Pos.el_3D(1,3), Pos.wr_3D(1,3)], ...
      'g-', 'LineWidth', 5);

% Gomito
plot3(Pos.el_3D(1,1), Pos.el_3D(1,2), Pos.el_3D(1,3), 'go', 'MarkerSize', 18, 'MarkerFaceColor', 'g', 'LineWidth', 2);
text(Pos.el_3D(1,1)-2, Pos.el_3D(1,2), Pos.el_3D(1,3)+2, 'GOMITO 0°', 'FontSize', 11, 'Color', 'g', 'FontWeight', 'bold');

% PIN INTERNO (magenta)
plot3([Pos.el_3D(1,1), Pos.gapel_3D(1,1)], [Pos.el_3D(1,2), Pos.gapel_3D(1,2)], [Pos.el_3D(1,3), Pos.gapel_3D(1,3)], ...
      'm-', 'LineWidth', 3);
plot3(Pos.gapel_3D(1,1), Pos.gapel_3D(1,2), Pos.gapel_3D(1,3), 'ms', 'MarkerSize', 16, 'MarkerFaceColor', 'm', 'LineWidth', 2);
text(Pos.gapel_3D(1,1), Pos.gapel_3D(1,2), Pos.gapel_3D(1,3)+2, 'PIN INT 0°', 'FontSize', 10, 'Color', 'm', 'FontWeight', 'bold');

% PIN ESTERNO (ciano)
plot3([Pos.el_3D(1,1), Pos.gapsw_3D(1,1)], [Pos.el_3D(1,2), Pos.gapsw_3D(1,2)], [Pos.el_3D(1,3), Pos.gapsw_3D(1,3)], ...
      'c-', 'LineWidth', 3);
plot3(Pos.gapsw_3D(1,1), Pos.gapsw_3D(1,2), Pos.gapsw_3D(1,3), 'cs', 'MarkerSize', 16, 'MarkerFaceColor', 'c', 'LineWidth', 2);
text(Pos.gapsw_3D(1,1), Pos.gapsw_3D(1,2), Pos.gapsw_3D(1,3)+2, 'PIN EST 0°', 'FontSize', 10, 'Color', 'c', 'FontWeight', 'bold');

% ===== CONFIGURAZIONE 2: θ = 90° (ROSSO) =====
% Braccio superiore
plot3([Pos.sh(1), Pos.el_3D(2,1)], [Pos.sh(2), Pos.el_3D(2,2)], [Pos.sh(3), Pos.el_3D(2,3)], ...
      'r-', 'LineWidth', 5);

% Avambraccio
plot3([Pos.el_3D(2,1), Pos.wr_3D(2,1)], [Pos.el_3D(2,2), Pos.wr_3D(2,2)], [Pos.el_3D(2,3), Pos.wr_3D(2,3)], ...
      'r-', 'LineWidth', 5);

% Gomito
plot3(Pos.el_3D(2,1), Pos.el_3D(2,2), Pos.el_3D(2,3), 'ro', 'MarkerSize', 18, 'MarkerFaceColor', 'r', 'LineWidth', 2);
text(Pos.el_3D(2,1)+2, Pos.el_3D(2,2), Pos.el_3D(2,3)+2, 'GOMITO 90°', 'FontSize', 11, 'Color', 'r', 'FontWeight', 'bold');

% PIN INTERNO (magenta scuro)
plot3([Pos.el_3D(2,1), Pos.gapel_3D(2,1)], [Pos.el_3D(2,2), Pos.gapel_3D(2,2)], [Pos.el_3D(2,3), Pos.gapel_3D(2,3)], ...
      'm--', 'LineWidth', 3);
plot3(Pos.gapel_3D(2,1), Pos.gapel_3D(2,2), Pos.gapel_3D(2,3), 'md', 'MarkerSize', 16, 'MarkerFaceColor', 'm', 'LineWidth', 2);
text(Pos.gapel_3D(2,1), Pos.gapel_3D(2,2), Pos.gapel_3D(2,3)-2, 'PIN INT 90°', 'FontSize', 10, 'Color', 'm', 'FontWeight', 'bold');

% PIN ESTERNO (ciano scuro)
plot3([Pos.el_3D(2,1), Pos.gapsw_3D(2,1)], [Pos.el_3D(2,2), Pos.gapsw_3D(2,2)], [Pos.el_3D(2,3), Pos.gapsw_3D(2,3)], ...
      'c--', 'LineWidth', 3);
plot3(Pos.gapsw_3D(2,1), Pos.gapsw_3D(2,2), Pos.gapsw_3D(2,3), 'cd', 'MarkerSize', 16, 'MarkerFaceColor', 'c', 'LineWidth', 2);
text(Pos.gapsw_3D(2,1), Pos.gapsw_3D(2,2), Pos.gapsw_3D(2,3)-2, 'PIN EST 90°', 'FontSize', 10, 'Color', 'c', 'FontWeight', 'bold');

% Asse di rotazione (spalla-polso)
plot3([Pos.sh(1), Pos.wr_init(1)], [Pos.sh(2), Pos.wr_init(2)], [Pos.sh(3), Pos.wr_init(3)], ...
      'b--', 'LineWidth', 2);

view(45, 20);
legend('Spalla', 'Polso (fisso)', 'Braccio 0°', 'Avambraccio 0°', 'Gomito 0°', ...
       'Pin INT→gomito 0°', 'Pin INT 0°', 'Pin EST→gomito 0°', 'Pin EST 0°', ...
       'Braccio 90°', 'Avambraccio 90°', 'Gomito 90°', ...
       'Pin INT→gomito 90°', 'Pin INT 90°', 'Pin EST→gomito 90°', 'Pin EST 90°', ...
       'Asse rotazione', 'Location', 'northeastoutside', 'FontSize', 8);

% ===== VERIFICA NUMERICA =====
fprintf('\n========== CONFRONTO 2 CONFIGURAZIONI ==========\n');
fprintf('Sistema: X=destra, Y=avanti, Z=alto | Braccio DESTRO\n\n');

fprintf('CONFIGURAZIONE 1 (θ = 0°):\n');
fprintf('  Gomito:     [%.2f, %.2f, %.2f]\n', Pos.el_3D(1,1), Pos.el_3D(1,2), Pos.el_3D(1,3));
fprintf('  Polso:      [%.2f, %.2f, %.2f]\n', Pos.wr_3D(1,1), Pos.wr_3D(1,2), Pos.wr_3D(1,3));
fprintf('  PIN INT:    [%.2f, %.2f, %.2f]\n', Pos.gapel_3D(1,1), Pos.gapel_3D(1,2), Pos.gapel_3D(1,3));
fprintf('  PIN EST:    [%.2f, %.2f, %.2f]\n\n', Pos.gapsw_3D(1,1), Pos.gapsw_3D(1,2), Pos.gapsw_3D(1,3));

fprintf('CONFIGURAZIONE 2 (θ = 90°):\n');
fprintf('  Gomito:     [%.2f, %.2f, %.2f]\n', Pos.el_3D(2,1), Pos.el_3D(2,2), Pos.el_3D(2,3));
fprintf('  Polso:      [%.2f, %.2f, %.2f] ← FISSO!\n', Pos.wr_3D(2,1), Pos.wr_3D(2,2), Pos.wr_3D(2,3));
fprintf('  PIN INT:    [%.2f, %.2f, %.2f]\n', Pos.gapel_3D(2,1), Pos.gapel_3D(2,2), Pos.gapel_3D(2,3));
fprintf('  PIN EST:    [%.2f, %.2f, %.2f]\n\n', Pos.gapsw_3D(2,1), Pos.gapsw_3D(2,2), Pos.gapsw_3D(2,3));

fprintf('DISTANZE (devono essere uguali nelle 2 configurazioni):\n');
fprintf('  Spalla-Gomito:    %.3f cm vs %.3f cm\n', ...
    norm(Pos.el_3D(1,:)-Pos.sh), norm(Pos.el_3D(2,:)-Pos.sh));
fprintf('  Gomito-Polso:     %.3f cm vs %.3f cm\n', ...
    norm(Pos.wr_3D(1,:)-Pos.el_3D(1,:)), norm(Pos.wr_3D(2,:)-Pos.el_3D(2,:)));
fprintf('  Gomito-PIN INT:   %.3f cm vs %.3f cm\n', ...
    norm(Pos.gapel_3D(1,:)-Pos.el_3D(1,:)), norm(Pos.gapel_3D(2,:)-Pos.el_3D(2,:)));
fprintf('  Gomito-PIN EST:   %.3f cm vs %.3f cm\n', ...
    norm(Pos.gapsw_3D(1,:)-Pos.el_3D(1,:)), norm(Pos.gapsw_3D(2,:)-Pos.el_3D(2,:)));
fprintf('===============================================\n\n');