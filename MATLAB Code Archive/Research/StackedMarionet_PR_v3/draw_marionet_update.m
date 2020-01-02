function draw_marionet_update(radius,angle,starting_pos,ending_pos,handles)

R = [cos(-pi/2+angle),-sin(-pi/2+angle);sin(-pi/2+angle),cos(-pi/2+angle)];
mar_norm  = R*[0;radius];
mar_pos = mar_norm + starting_pos;
dist_startmar = [starting_pos, mar_pos];
dist_endmar = [ending_pos, mar_pos];

set(handles(1),'XData',dist_startmar(1,:);
set(handles(1),'YData',dist_startmar(2,:);
set(handles(2),'XData',mar_pos(1));
set(handles(2),'YData',mar_pos(2));
set(handles(3),'XData',dist_end_mar(1,:));
set(handles(3),'YData',dist_end_mar(2,:));

end
