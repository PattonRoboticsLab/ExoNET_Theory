% Function that evaluate the positions of the body and of the CenterMass(CM) in 3D
function [Pos, RotMatrix, Jacobian_ShEl] = forwardKin3D(PHIs, Bod, Exo, Pos, mode) 

L1 = Bod.L(1);  L2 = Bod.L(2);  % Length of the upper arm and forearm

%% Evaluate all the points of the arm in different planes for ELEVATION
Pos.sh = Exo.shoulder;       phi12  = PHIs.el(:,1) + PHIs.el(:,2); % Sum phi's 

% Position of the upper arm and its center of mass
Pos.el_init = Pos.sh + L1 * [ cos(PHIs.el(:,3)).*cos(PHIs.el(:,1)),  sin(PHIs.el(:,3)).*cos(PHIs.el(:,1)),  sin(PHIs.el(:,1)) ];

% Position of the forearm and its center of mass
Pos.el2wr =  L2 * [ cos(PHIs.el(:,3)).*cos(phi12),  sin(PHIs.el(:,3)).*cos(phi12),  sin(phi12) ] ;        

Pos.wr_init = Pos.el_init + Pos.el2wr; % Positon of the wrist before rotation

%% Choose the mode of the function
if strcmp(mode, 'EA')
    RotMatrix = []; 
else
    % Evaluate the positions of the elbow during internal rotation (swivel)
% keeping the wrist position fixed
RotMatrix = []; % Initialize RotMatrix
for i = 1:size(Pos.wr_init, 1)
    % Define the axis of rotation: from shoulder to wrist (fixed)
    axis_unit = (Pos.wr_init(i,:) - Pos.sh) ./ norm(Pos.wr_init(i,:) - Pos.sh);
    
    % Calculate the initial elbow position relative to shoulder-wrist axis
    sh_to_el = Pos.el_init(i,:) - Pos.sh;
    
    % Project this vector onto the shoulder-wrist axis
    projection_length = dot(sh_to_el, axis_unit);
    projection_point = Pos.sh + projection_length * axis_unit;
    
    % Perpendicular component (from axis to elbow) - questa è la posizione a θ=0
    radial_vector = Pos.el_init(i,:) - projection_point;
    radial_distance = norm(radial_vector);
    radial_unit = radial_vector ./ radial_distance;
    
    % IMPORTANTE: Per rotazione ANTIORARIA vista dalla spalla verso il polso
    % Il tangent_unit deve essere orientato correttamente
    % Per braccio DESTRO: cross(radial_unit, axis_unit) dà rotazione antioraria
    tangent_unit = cross(radial_unit, axis_unit);
    tangent_unit = tangent_unit ./ norm(tangent_unit);
    
    % Rotate the elbow around the shoulder-wrist axis
    for j = 1:length(Exo.theta)
        % Wrist position remains FIXED
        Pos.wr_3D(i,j,:) = Pos.wr_init(i,:);
        
        % Elbow rotates COUNTER-CLOCKWISE (when looking from shoulder to wrist)
        % θ=0° → posizione iniziale (radial_unit)
        % θ>0° → rotazione antioraria
        current_elbow = projection_point + ...
            radial_distance * (cos(Exo.theta(j)) * radial_unit + sin(Exo.theta(j)) * tangent_unit);
        Pos.el_3D(i,j,:) = current_elbow;
        
        % === PIN SOLIDALI CON L'AVAMBRACCIO - ANATOMICAMENTE CORRETTI ===
        
        % Asse avambraccio (gomito -> polso)
        forearm_axis = (Pos.wr_init(i,:) - current_elbow) ./ norm(Pos.wr_init(i,:) - current_elbow);
        
        % Asse braccio superiore (spalla -> gomito)
        upper_arm_axis = (current_elbow - Pos.sh) ./ norm(current_elbow - Pos.sh);
        
        % gapel: PIN INTERNO - verso l'angolo di flessione del gomito
        bisector = forearm_axis - upper_arm_axis;
        bisector_unit = bisector ./ norm(bisector);
        Pos.gapel_3D(i,j,:) = current_elbow + Bod.gap * bisector_unit;
        
        % gapsw: PIN ESTERNO - perpendicolare a gapel e all'avambraccio
        lateral_unit = cross(forearm_axis, bisector_unit);
        lateral_unit = lateral_unit ./ norm(lateral_unit);
        Pos.gapsw_3D(i,j,:) = current_elbow + Bod.gap * lateral_unit;

    end % end for j
end % end for i

%% Find the position of the elbow and of the gap of the swivel angle
tau_numb = 1; % Initialize the number of torque forces for the vector
for k = 1:size(Pos.wr_3D, 1) % For each position of the elbow squeeze the pos of the wr and offsets
    wrist_position = squeeze( Pos.wr_3D(k,:,:) );
    elbow_position = squeeze( Pos.el_3D(k,:,:) );  % ← AGGIUNTO: prendi gomito ruotato!
    gapsw = squeeze( Pos.gapsw_3D(k,:,:) );
    gapel = squeeze( Pos.gapel_3D(k,:,:) );
    
    for j = 1:size(Pos.wr_3D, 2) % For each position of the elbow, store positions

        Pos.wrSwivel(tau_numb,:) = wrist_position(j,:);
        Pos.elbowSwivel(tau_numb,:) = elbow_position(j,:); % ← CORRETTO: usa gomito ruotato!
        Pos.gapel(tau_numb,:) = gapel(j,:);
        Pos.gapsw(tau_numb,:) = gapsw(j,:);
        tau_numb = tau_numb + 1;

    end % end for cycle
end % end for cycle


%% --------------------------------------------------------------

%% PLOT ANIMAZIONE DEI PUNTI FINALI
% Da inserire DOPO il ciclo che crea Pos.wrSwivel, Pos.elbowSwivel, Pos.gapel, Pos.gapsw

% ===== SETUP FIGURA =====
% figure('Position', [100, 100, 1200, 800]);
% hold on; grid on; axis equal;
% xlabel('X [cm] - laterale');
% ylabel('Y [cm] - anteriore');
% zlabel('Z [cm] - superiore');
% title('Animazione Configurazioni Braccio', 'FontSize', 14, 'FontWeight', 'bold');
% view(45, 20);
% 
% % Trova limiti automatici basati sui dati
% all_points = [Pos.sh; Pos.wrSwivel; Pos.elbowSwivel; Pos.gapel; Pos.gapsw];
% x_range = max(all_points(:,1)) - min(all_points(:,1));
% y_range = max(all_points(:,2)) - min(all_points(:,2));
% z_range = max(all_points(:,3)) - min(all_points(:,3));
% % Usa margine proporzionale (10% del range, minimo 0.05m)
% margin_x = max(0.05, 0.1 * x_range);
% margin_y = max(0.05, 0.1 * y_range);
% margin_z = max(0.05, 0.1 * z_range);
% xlim([min(all_points(:,1))-margin_x, max(all_points(:,1))+margin_x]);
% ylim([min(all_points(:,2))-margin_y, max(all_points(:,2))+margin_y]);
% zlim([min(all_points(:,3))-margin_z, max(all_points(:,3))+margin_z]);
% 
% % ===== ELEMENTI FISSI =====
% % Spalla
% plot3(Pos.sh(1), Pos.sh(2), Pos.sh(3), 'ko', 'MarkerSize', 20, 'MarkerFaceColor', 'k', 'LineWidth', 2);
% text(Pos.sh(1)-2, Pos.sh(2), Pos.sh(3)+3, 'SPALLA', 'FontSize', 11, 'FontWeight', 'bold');
% 
% % ===== ELEMENTI ANIMATI =====
% % Braccio superiore (spalla-gomito)
% h_upper = plot3([0, 0], [0, 0], [0, 0], 'r-', 'LineWidth', 5);
% 
% % Avambraccio (gomito-polso)
% h_forearm = plot3([0, 0], [0, 0], [0, 0], 'b-', 'LineWidth', 5);
% 
% % Gomito
% h_elbow = plot3(0, 0, 0, 'ro', 'MarkerSize', 18, 'MarkerFaceColor', 'r', 'LineWidth', 2);
% 
% % Polso
% h_wrist = plot3(0, 0, 0, 'bo', 'MarkerSize', 18, 'MarkerFaceColor', 'b', 'LineWidth', 2);
% 
% % Pin interno
% h_pin_int_line = plot3([0, 0], [0, 0], [0, 0], 'm-', 'LineWidth', 3);
% h_pin_int = plot3(0, 0, 0, 'mo', 'MarkerSize', 16, 'MarkerFaceColor', 'm', 'LineWidth', 2);
% 
% % Pin esterno
% h_pin_ext_line = plot3([0, 0], [0, 0], [0, 0], 'c-', 'LineWidth', 3);
% h_pin_ext = plot3(0, 0, 0, 'co', 'MarkerSize', 16, 'MarkerFaceColor', 'c', 'LineWidth', 2);
% 
% % Testo
% h_text = text(0, 0, 0, '', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');
% 
% legend('Spalla', 'Braccio superiore', 'Avambraccio', 'Gomito', 'Polso', ...
%        'Verso PIN INT', 'PIN INT', 'Verso PIN EST', 'PIN EST', ...
%        'Location', 'northeastoutside', 'FontSize', 9);
% 
% % ===== ANIMAZIONE =====
% num_configs = size(Pos.wrSwivel, 1);
% pause_time = 0.2; % Secondi tra configurazioni
% 
% fprintf('\n=== ANIMAZIONE CONFIGURAZIONI ===\n');
% fprintf('Numero totale configurazioni: %d\n', num_configs);
% fprintf('Premi Ctrl+C per interrompere\n\n');
% 
% for i = 1:num_configs
%     % Posizioni correnti
%     elbow = Pos.elbowSwivel(i,:);
%     wrist = Pos.wrSwivel(i,:);
%     gapel = Pos.gapel(i,:);
%     gapsw = Pos.gapsw(i,:);
% 
%     % Aggiorna braccio superiore
%     set(h_upper, 'XData', [Pos.sh(1), elbow(1)], ...
%                  'YData', [Pos.sh(2), elbow(2)], ...
%                  'ZData', [Pos.sh(3), elbow(3)]);
% 
%     % Aggiorna avambraccio
%     set(h_forearm, 'XData', [elbow(1), wrist(1)], ...
%                    'YData', [elbow(2), wrist(2)], ...
%                    'ZData', [elbow(3), wrist(3)]);
% 
%     % Aggiorna gomito
%     set(h_elbow, 'XData', elbow(1), 'YData', elbow(2), 'ZData', elbow(3));
% 
%     % Aggiorna polso
%     set(h_wrist, 'XData', wrist(1), 'YData', wrist(2), 'ZData', wrist(3));
% 
%     % Aggiorna pin interno
%     set(h_pin_int_line, 'XData', [elbow(1), gapel(1)], ...
%                         'YData', [elbow(2), gapel(2)], ...
%                         'ZData', [elbow(3), gapel(3)]);
%     set(h_pin_int, 'XData', gapel(1), 'YData', gapel(2), 'ZData', gapel(3));
% 
%     % Aggiorna pin esterno
%     set(h_pin_ext_line, 'XData', [elbow(1), gapsw(1)], ...
%                         'YData', [elbow(2), gapsw(2)], ...
%                         'ZData', [elbow(3), gapsw(3)]);
%     set(h_pin_ext, 'XData', gapsw(1), 'YData', gapsw(2), 'ZData', gapsw(3));
% 
%     % Aggiorna testo
%     set(h_text, 'Position', [elbow(1)+2, elbow(2), elbow(3)+3], ...
%                 'String', sprintf('Config %d/%d', i, num_configs));
% 
%     % Refresh
%     drawnow;
%     pause(pause_time);
% 
%     % Feedback
%     if mod(i, 10) == 0 || i == num_configs
%         fprintf('Configurazione %d/%d\n', i, num_configs);
%     end
% end
% 
% fprintf('\n=== COMPLETATO ===\n\n');

%% -------------------------------------------------

end % End of the function

%% Evaluate the Jacobian
Jacobian_ShEl = [];
% syms phi1 phi2 phi3 L1 L2 real
% 
% el = L1 * [cos(phi3)*cos(phi1); sin(phi3)*cos(phi1); sin(phi1)];
% phi12 = phi1 + phi2;
% wr = el + L2 * [cos(phi3)*cos(phi12); sin(phi3)*cos(phi12); sin(phi12)];
% 
% % J = [L_upper, L_fore] * [   cos(phi3) * cos(phi1),        sin(phi3) * cos(phi1),       sin(phi1);
% %                          cos(phi3) * cos(phi1 + phi2), sin(phi3) * cos(phi1+ phi2), sin(phi1 + phi2) ];
% Jacobians = jacobian(wr, [phi1, phi2 phi3]);
% 
% for i = 1:size(PHIs.J, 1)  
%     Jacobians_sub = subs(Jacobians, {phi1, phi2, phi3 L_upper, L_fore}, {PHIs.J(i, 1), PHIs.J(i, 2),q, L1, L2});
%     Jacobian_ShEl = [Jacobian_ShEl, Jacobians_sub];
% end
% Jacobian_ShEl = double(Jacobian_ShEl);

%% Helpfull to take everything into GPU
% Jacobian_ShEL = gpuArray(double(Jacobian_ShEl));

% %% Create the Jacobian for the swivel angle
% Jacobian_swivel = [];
% syms elr sw_angle 
% Jacob = elr * [ cos(sw_angle), sin(sw_angle), 0 ];  % Rotation in the plane
% %Jacob = gpuArray (Jacob);
% Jacobians_sw = jacobian( Jacob, sw_angle );
% Jacobian_swivel = [];
% for j = 1:length(theta) 
%     Jacobians_SwSh = subs(Jacobians_sw, {elr, sw_angle}, {elbow_radius, theta(j)});
%     Jacobian_swivel = [Jacobian_swivel, Jacobians_SwSh];
% end
% Jacobian_swivel = double(Jacobian_swivel);
% %Jacobian_swivel = gpuArray(double(Jacobian_swivel));