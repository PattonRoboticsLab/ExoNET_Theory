function handles = draw_marionet_handles(radius, angle, starting_pos, ending_pos)
   
    R = [cos(-pi/2+angle), -sin(-pi/2+angle); sin(-pi/2+angle), cos(-pi/2+angle)];

    mar_norm = R*[0; radius];
    mar_pos = mar_norm+starting_pos;
    dist_startmar = [starting_pos, mar_pos];
    dist_endmar = [ending_pos, mar_pos];
    
    hold on 
    pl = plot(dist_startmar(1, :), dist_startmar(2, :), 'm', 'linewidth', 1);
    hold on
    sc = scatter(mar_pos(1), mar_pos(2), 'k', 'filled');
    hold on
    pl2 = plot(dist_endmar(1, :), dist_endmar(2, :), 'r', 'linewidth', 1);
    handles = [pl, sc, pl2];
    
end
