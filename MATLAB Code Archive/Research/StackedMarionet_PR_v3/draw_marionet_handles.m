function handles = draw_marionet_handles(radius, angle, starting_pos, ending_pos)

R = [cos(-pi/2+angle),-sin(-pi/2+angle);sin(-pi/2+angle),cos(-pi/2+angle)];
mar_norm  = R*[0;radius];
mar_pos = mar_norm + starting_pos;
dist_startmar = [starting_pos, mar_pos];
dist_endmar = [ending_pos, mar_pos];

hold on
p1 = plot(dist_startmar(1,:),dist_startmar(2,:),'b','linewidth',2);
hold on
sc = scatter(mar_pos(1), mar_pos(2), 'k','filled');
hold on
p12 = plot(dist_endmar(1,:), dist_endmar(2,:),'g','linewidth',2);
handles = [p1,sc,p12];
end
