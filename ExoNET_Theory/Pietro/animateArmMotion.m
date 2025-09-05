function animateArmMotion(robot, q)
% Animation of robotic arm given a model and a sequence of configurations q
% q: [N x 4] matrix of joint configurations

    % === Stima lunghezza massima del braccio per autoscale
    T_end = getTransform(robot, q(1,:), robot.BodyNames{end});
    pos_end = T_end(1:3, 4);
    max_reach = norm(pos_end); % margine per visibilità
    disp(['Max Reach stimato: ', num2str(max_reach), ' m']); % DEBUG

    % === Setup figura e assi
    figure('Name','Animazione braccio 3D');
    ax = axes;
    view(ax, 2); % Equivalente a view(ax, 0, 90); 
    grid(ax, 'on'); axis(ax, 'equal');
    xlabel(ax, 'X (m) - Forward');
    ylabel(ax, 'Y (m) - Lateral');
    zlabel(ax, 'Z (m) - Vertical');
    xlim(ax, [-max_reach*0.6, max_reach*0.6]);
    ylim(ax, [-max_reach*0.6, max_reach*0.6]);
    zlim(ax, [-max_reach*0.6, max_reach*0.3]);
    disp(['Assi impostati: X [', num2str(-max_reach*0.6), ', ', num2str(max_reach*0.6), ']']); % DEBUG

    % === Messaggio iniziale
    fprintf('\n=== STARTING RIGHT ARM ANIMATION ===\n\n');
    disp(['Numero di frame totali: ', num2str(size(q, 1))]); % DEBUG

    % === Loop di animazione
    for i = 1:size(q, 1)
        % Pulisce solo il contenuto grafico senza cancellare gli assi
        cla(ax);

        % Mostra il robot nella configurazione corrente
        show(robot, q(i,:), 'Frames','on', 'Visuals','on', 'Parent', ax);

        % Luci (fisse, non accumulate)
        camlight(ax, 'headlight');

        % Titolo con i valori angolari
        title(ax, sprintf('Frame %d/%d - Abd: %.1f° Elev: %.1f° Rot: %.1f° Elbow: %.1f°', ...
            i, size(q, 1), q(i,1)*180/pi, q(i,2)*180/pi, q(i,3)*180/pi, q(i,4)*180/pi));

        drawnow;
        pause(0.8); % tempo tra i frame (regolabile)
        % disp(['Frame ', num2str(i), ' disegnato.']); % DEBUG (potrebbe rallentare molto)
    end

    fprintf('=== ANIMAZIONE COMPLETATA ===\n');
end