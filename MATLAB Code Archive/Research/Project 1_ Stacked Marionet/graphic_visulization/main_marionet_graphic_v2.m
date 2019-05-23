clear all
close all
clc

%n = 100;
L_upper = 0.35;
L_fore = 0.26;

g = 9.81;   %gravity constant
m_tot = 70; %total body mass in Kg
p_upper = (2.71/100)*m_tot*g; %mass upper arm
p_fore = (1.62/100)*m_tot*g;  %mass forearm
p_hand = (0.61/100)*m_tot*g;  %mass hand
param = load('opt_param.mat'); %row vector
n = length(param.opt_param)/2;

radius = param.opt_param(1, n+1:end);
angle = param.opt_param(1, 1:n);

wh = p_hand*g; %weight of the hand
phi1 = -pi/4;
k = 10;
phi2 = linspace(0, 2*pi/3, k);

sh_pos = [0; 0];
el_pos = sh_pos + [L_upper*cos(phi1); L_upper*(sin(phi1))];
dist_se = [sh_pos, el_pos];

sh_el = [(sh_pos(1)+el_pos(1))/2; (sh_pos(2)+el_pos(2))/2];

alpha = phi2(1)+phi1;
wr_pos = el_pos + [L_fore*cos(alpha); L_fore*(sin(alpha))];
el_wr = [(el_pos(1)+wr_pos(1))/2; (el_pos(2)+wr_pos(2))/2];
dist_ew = [el_pos, wr_pos];

head_pos = [0; .1];
bas_pos = [0; -.35];
dist_headbas = [head_pos, bas_pos];
nose_pos = [.07; .1];
dist_noshead = [head_pos, nose_pos];

figure(1)

plot(dist_headbas(1, :), dist_headbas(2, :), 'g', 'linewidth', 5);
hold on
plot(dist_se(1, :), dist_se(2, :), 'g', 'linewidth', 5);
grid on
hold on
scatter(head_pos(1), head_pos(2), 2000, 'k', 'filled');
hold on
scatter(el_pos(1), el_pos(2), 100, 'k', 'filled');
hold on
scatter(sh_pos(1), sh_pos(2), 100, 'k', 'filled');
hold on
scatter(sh_el(1), sh_el(2), 'k', 'filled');
hold on
plot(dist_noshead(1, :), dist_noshead(2, :), 'k', 'linewidth', 5);
hold on
a = plot(dist_ew(1, :), dist_ew(2, :), 'g', 'linewidth', 5);
hold on
b = scatter(wr_pos(1), wr_pos(2), 100, 'k', 'filled');
hold on
c = scatter(el_wr(1), el_wr(2), 'k', 'filled');
hold on
handles = [];


axis([-.05, .55, -0.5, .15])

for j = 2:k
 
    alpha = phi2(j)+phi1;
    wr_pos = el_pos + [L_fore*cos(alpha); L_fore*(sin(alpha))];
    el_wr = [(el_pos(1)+wr_pos(1))/2; (el_pos(2)+wr_pos(2))/2];
    dist_ew = [el_pos, wr_pos];
    hold on
    for i = 1:n
        f = draw_marionet_handles(radius(i), angle(i), el_pos, wr_pos);
        handles = [handles; f];
    end
    set(a, 'XData', dist_ew(1, :));
    set(a, 'YData', dist_ew(2, :));
    set(b, 'XData', wr_pos(1));
    set(b, 'YData', wr_pos(2));
    set(c, 'XData', el_wr(1));
    set(c, 'YData', el_wr(2));
    drawnow  

    axis([-.05, .55, -0.5, .15])
    pause(.000001)
    if (j~=k)
        delete(handles);
    end
    
end