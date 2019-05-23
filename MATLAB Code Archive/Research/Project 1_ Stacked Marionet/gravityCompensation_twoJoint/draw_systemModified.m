function draw_systemModified(shoulder_pos, elbow_pos, wrist_pos, R_el)%, force_vector)
    
    %weights: [upper arm weight, fore arm weight, hand weight]
    %lenghts: [upper arm length, fore_arm length]
    %force_vector: [elbow_force, shoulder_force]
    
    head_pos = [0; .2];
    bas_pos = [0; -.7];
    nose_pos1 = [.13; .17];
    nose_pos2 = [0; .17];
    eye_pos = [.06, .21];
    dist_headbas = [head_pos, bas_pos];
    dist_se = [shoulder_pos, elbow_pos];
    dist_noshead = [head_pos, nose_pos1];
    dist_ew = [elbow_pos, wrist_pos];
    dist_nose = [nose_pos1, nose_pos2];
    
    sh_el = [(shoulder_pos(1)+elbow_pos(1))/2; (shoulder_pos(2)+elbow_pos(2))/2];
    el_wr = [(elbow_pos(1)+wrist_pos(1))/2; (elbow_pos(2)+wrist_pos(2))/2];

    retta1 = [.4; 0];
    retta2 = [.5; 0];
    retta2_rot = R_el*retta2;
    figure(1);
    plot(dist_headbas(1, :), dist_headbas(2, :), 'm', 'linewidth', 5);
    hold on
    plot(dist_se(1, :), dist_se(2, :), 'm', 'linewidth', 5);
    hold on
    plot([shoulder_pos(1), retta1(1)], [shoulder_pos(2), retta1(2)], 'k-.')
    hold on
    plot([shoulder_pos(1), retta2_rot(1)], [shoulder_pos(2), retta2_rot(2)], 'k-.')
    hold on
    plot(dist_ew(1, :), dist_ew(2, :), 'm', 'linewidth', 5);
    hold on
    plot(dist_noshead(1, :), dist_noshead(2, :), 'm', 'linewidth', 5);
    hold on
    plot(dist_nose(1, :), dist_nose(2, :), 'm', 'linewidth', 5);
    hold on
    scatter(head_pos(1), head_pos(2), 2000, 'm', 'filled');
    hold on
    scatter(elbow_pos(1), elbow_pos(2), 100, 'k', 'filled');
    hold on
    scatter(shoulder_pos(1), shoulder_pos(2), 100, 'k', 'filled');
    hold on
    scatter(wrist_pos(1), wrist_pos(2), 100, 'k', 'filled');
    hold on
    
    scatter(eye_pos(1), eye_pos(2), 'k', 'filled') 
    hold on
   
    axis equal
    
    txt_3 = 'S';
    txt_4 = 'E';
    txt_5 = 'W';
    text_pos1 = shoulder_pos + [-.1; 0];
    text_pos2 = elbow_pos + [-.02; -.07];
    text_pos3 = wrist_pos + [-.02; .07];
    t1 = text(text_pos1(1), text_pos1(2), txt_3);
    t2 = text(text_pos2(1), text_pos2(2), txt_4);
    t5 = text(text_pos3(1), text_pos3(2), txt_5);
    set(t1, 'FontSize', 15);
    set(t2, 'FontSize', 15);
    set(t5, 'FontSize', 15);
    
end