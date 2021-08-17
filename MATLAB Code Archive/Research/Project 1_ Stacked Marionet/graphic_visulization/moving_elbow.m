clear all
close all
clc

n = 50;
L_upper = 0.35;
L_fore = 0.26;

g = 9.81;   %gravity constant
m_tot = 70; %total body mass in Kg
m_upper = (2.71/100)*m_tot; %mass uppòer arm
m_fore = (1.62/100)*m_tot;  %mass forearm
m_hand = (0.61/100)*m_tot;  %mass hand

wh = m_hand*g; %weight of the hand

phi1 = -pi/4;
k = 10;
phi2 = linspace(0, 2*pi/3, k);

R1 = [cos(-phi1), -sin(-phi1); sin(-phi1), cos(-phi1)];

sh_pos = [0; 0];
el_pos = sh_pos + [L_upper*cos(phi1); L_upper*(sin(phi1))];
dist_se = [sh_pos, el_pos];
sh_el = [(sh_pos(1)+el_pos(1))/2; (sh_pos(2)+el_pos(2))/2];

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
arrow(sh_el, sh_el-[0; .1*m_upper]);
hold on
plot(dist_noshead(1, :), dist_noshead(2, :), 'k', 'linewidth', 5);

for j = 1:k
    %phi2 = pi/6;
    alpha = phi2(j)+phi1;
    R2 = [cos(+alpha), -sin(+alpha); sin(+alpha), cos(+alpha)];
    wr_pos = el_pos + [L_fore*cos(alpha); L_fore*(sin(alpha))];
    el_wr = [(el_pos(1)+wr_pos(1))/2; (el_pos(2)+wr_pos(2))/2];
    dist_ew = [el_pos, wr_pos];
    
    limit_min1 = 0;
    limit_max1 = L_fore;
    fun_half2_el = @(x, phi2)-wh.*x.*cos(phi2);
    fun_half1_el = @(x, phi2)-(wh+m_fore*g).*x.*cos(phi2);
    
    armel = (linspace(limit_min1, limit_max1, n));
    passo = limit_max1/(n-1);
    
    a = plot(dist_ew(1, :), dist_ew(2, :), 'g', 'linewidth', 5);
    hold on
    b = scatter(wr_pos(1), wr_pos(2), 100, 'k', 'filled');
    hold on
    c = scatter(el_wr(1), el_wr(2), 'k', 'filled');
    hold on
    pt_elb = el_wr-[0; .2*m_fore];
    d = arrow(el_wr, pt_elb);
    hold on
    e = arrow(wr_pos, wr_pos-[0; .1*m_hand]);
    
    reaction_el = zeros(1, n);
    for i = 1:n

        if (armel(i)<=L_fore/2)
            reaction_el(i) = fun_half1_el(armel(i), phi2(j));  
        elseif (armel(i)>L_fore/2)
            reaction_el(i) = fun_half2_el(armel(i), phi2(j));
        end

    end
    max_el = max(-reaction_el);
    min_el = min(-reaction_el);
    reaction_el_norm = (reaction_el-min_el).*0.1./(max_el-min_el);

    el_pos_old = el_pos;
    handles = [];
    for i = 1:n

        if (i == 1)
            el_pos_new = el_pos_old;
        else
            el_pos_new = el_pos_old + [passo*cos(alpha); passo*sin(alpha)];
        end
        if (atan2(wr_pos(2)-el_pos(2), wr_pos(1)-el_pos(1)) >= pi/2)
            j
            arr_2 = (el_pos_new+R2*[0; reaction_el_norm(i)]);
        else
            arr_2 = (el_pos_new+R2*[0; reaction_el_norm(i)]);
        end
        l = line([el_pos_new(1), arr_2(1)], [el_pos_new(2), arr_2(2)], 'Color', 'r');
        hold on
        handles = [handles, l];
        el_pos_old = el_pos_new;

    end
    axis([-.05, .65, -0.5, .3])
    drawnow
    if (j ~= k)
        delete(a)
        delete(b)
        delete(c)
        delete(e)
        delete(d)
        delete(handles)
    end
end
