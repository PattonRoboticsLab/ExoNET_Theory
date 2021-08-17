function draw_system(sh_pos, el_pos, wr_pos)
    
    global fig2
    
    head_pos = [0; .2];
    bas_pos = [0; -.7];
    nose_pos1 = [.1; .17];
    nose_pos2 = [0; .17];
    eye_pos = [.05, .21];
    dist_headbas = [head_pos, bas_pos];
    dist_se = [sh_pos, el_pos];
    dist_noshead = [head_pos, nose_pos1];
    dist_ew = [el_pos, wr_pos];
    dist_nose = [nose_pos1, nose_pos2];

    figure();
    plot(dist_headbas(1, :), dist_headbas(2, :), 'm', 'linewidth', 5);
    grid on
    hold on
    plot(dist_se(1, :), dist_se(2, :), 'm', 'linewidth', 5);
    hold on
    plot(dist_ew(1, :), dist_ew(2, :), 'm', 'linewidth', 5);
    hold on
    plot(dist_noshead(1, :), dist_noshead(2, :), 'm', 'linewidth', 5);
    hold on
    plot(dist_nose(1, :), dist_nose(2, :), 'm', 'linewidth', 5);
    hold on
    scatter(head_pos(1), head_pos(2), 2000, 'm', 'filled');
    hold on
    scatter(el_pos(1), el_pos(2), 100, 'k', 'filled');
    hold on
    scatter(sh_pos(1), sh_pos(2), 100, 'k', 'filled');
    hold on
    scatter(wr_pos(1), wr_pos(2), 100, 'k', 'filled');
    hold on
    scatter(eye_pos(1), eye_pos(2), 'k', 'filled')
    
end