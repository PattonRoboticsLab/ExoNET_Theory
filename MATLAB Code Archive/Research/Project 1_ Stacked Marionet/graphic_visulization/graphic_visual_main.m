clear all
close all
clc

n = 100;
L_upper = 0.35;
L_fore = 0.26;

g = 9.81;   %gravity constant
m_tot = 70; %total body mass in Kg
m_upper = (2.71/100)*m_tot; %mass uppòer arm
m_fore = (1.62/100)*m_tot;  %mass forearm
m_hand = (0.61/100)*m_tot;  %mass hand

wh = m_hand*g; %weight of the hand
phi1 = -pi/4;
phi2 = pi/6;
alpha = phi2+phi1;
R1 = [cos(-phi1), -sin(-phi1); sin(-phi1), cos(-phi1)];
R2 = [cos(-alpha), -sin(-alpha); sin(-alpha), cos(-alpha)];

sh_pos = [0, 0];
el_pos = sh_pos + [L_upper*cos(phi1), L_upper*(sin(phi1))];
dist_se = [sh_pos; el_pos];
wr_pos = el_pos + [L_fore*cos(alpha), L_fore*(sin(alpha))];

limit_min1 = 0;
limit_max1 = L_fore;
fun_half2_el = @(x)-wh.*x.*cos(phi2);
fun_half1_el = @(x)-(wh+m_fore*g).*x.*cos(phi2);
armel = linspace(limit_min1, limit_max1, n);
passo = limit_max1/(n-1);

el_wr = [(el_pos(1)+wr_pos(1))/2, (el_pos(2)+wr_pos(2))/2];
sh_el = [(sh_pos(1)+el_pos(1))/2, (sh_pos(2)+el_pos(2))/2];

%% Elbow
figure(1)
plot(dist_se(:, 1), dist_se(:, 2), 'g', 'linewidth', 5);
grid on
hold on
dist_ew = [el_pos; wr_pos];
plot(dist_ew(:, 1), dist_ew(:, 2), 'g', 'linewidth', 5);
hold on
scatter(el_pos(1), el_pos(2), 100, 'k', 'filled');
hold on
scatter(sh_pos(1), sh_pos(2), 100, 'k', 'filled');
hold on
scatter(wr_pos(1), wr_pos(2), 100, 'k', 'filled');
hold on
scatter(el_wr(1), el_wr(2), 'k', 'filled');
hold on
scatter(sh_el(1), sh_el(2), 'k', 'filled');
hold on
arrow(sh_el, sh_el-[0, .1*m_upper]);
hold on
arrow(el_wr, el_wr-[0, .1*m_fore]);
hold on
arrow(wr_pos, wr_pos-[0, .1*m_hand]);
axis equal

reaction_el = zeros(1, n);
for i = 1:n
    
    if (armel(i)<=L_fore/2)
        reaction_el(i) = fun_half1_el(armel(i));  
    elseif (armel(i)>L_fore/2)
        reaction_el(i) = fun_half2_el(armel(i));
    end
    
end
max_el = max(-reaction_el);
min_el = min(-reaction_el);
reaction_el_norm = (reaction_el-min_el).*0.1./(max_el-min_el);

el_pos_old = el_pos;
for i = 1:n
    
    if (i == 1)
        el_pos_new = el_pos_old;
    else
        el_pos_new = el_pos_old + [passo*cos(alpha), passo*sin(alpha)];
    end
    arr_2 = (el_pos_new+[0, reaction_el_norm(i)]*R2);
    line([el_pos_new(1), arr_2(1)], [el_pos_new(2), arr_2(2)], 'Color', 'r')
    hold on
    el_pos_old = el_pos_new;
    
end

%% Shoulder
figure(2)
plot(dist_se(:, 1), dist_se(:, 2), 'g', 'linewidth', 5);
grid on
hold on
wr_pos = el_pos + [L_fore*cos(alpha), L_fore*(sin(alpha))];
dist_ew = [el_pos; wr_pos];
plot(dist_ew(:, 1), dist_ew(:, 2), 'g', 'linewidth', 5);
hold on
scatter(el_pos(1), el_pos(2), 100, 'k', 'filled');
hold on
scatter(sh_pos(1), sh_pos(2), 100, 'k', 'filled');
hold on
scatter(wr_pos(1), wr_pos(2), 100, 'k', 'filled');
hold on
scatter(el_wr(1), el_wr(2), 'k', 'filled');
hold on
scatter(sh_el(1), sh_el(2), 'k', 'filled');
hold on
arrow(sh_el, sh_el-[0, .1*m_upper]);
hold on
arrow(el_wr, el_wr-[0, .1*m_fore]);
hold on
arrow(wr_pos, wr_pos-[0, .1*m_hand]);
axis equal

fun_half2_es = @(x2) -wh*(x2*cos(phi2)+L_upper*cos(phi1));
fun_half1_es = @(x2) -wh*(2*x2*cos(phi2)+L_upper*cos(phi1))-m_fore*g*(L_upper*cos(phi1)+x2*cos(phi2));
armse_1 = linspace(limit_min1, limit_max1, n);

reaction_es = zeros(1, n);
for i = 1:n
    
    if (armse_1(i)<=L_fore/2)
        reaction_es(i) = fun_half1_es(armse_1(i));  
    elseif (armel(i)>L_fore/2)
        reaction_es(i) = fun_half2_es(armse_1(i));
    end
    
end
limit_min2 = 0;
limit_max2 = L_upper;
fun_half1_sh = @(x) -wh*(x*cos(phi1))-m_fore*g*(x*cos(phi1));
fun_half2_sh = @(x) -m_upper*g*x*cos(phi1)-wh*(x*cos(phi1))-m_fore*g*(x*cos(phi1));
armsh = linspace(limit_min2, limit_max2, n);
passo2 = limit_max2/(n-1);

reaction_sh = zeros(1, n);
for i = 1:n
    
    if (armsh(i)<=L_upper/2)
        reaction_sh(i) = fun_half2_sh(armsh(i));  
    elseif (armsh(i)>L_upper/2)
        reaction_sh(i) = fun_half1_sh(armsh(i));
    end
    
end

max_sh = max(-reaction_sh);
min_sh = min(-reaction_sh);
max_es = max(-reaction_es);
min_es = min(-reaction_es);

if(max_es > max_sh)
    max_1 = max_es;
elseif(max_sh > max_es)
    max_1 = max_sh;
end

if(min_es < min_sh)
    min_1 = min_es;
elseif(min_sh < min_es)
    min_1 = min_sh;
end    
reaction_es_norm = (reaction_es-min_1).*0.1./(max_1-min_1);

reaction_sh_norm = (reaction_sh-min_1).*0.1./(max_1-min_1);

sh_pos_old = sh_pos;
for i = 1:n
    
    if (i == 1)
        sh_pos_new = sh_pos_old;
    else
        sh_pos_new = sh_pos_old + [passo2*cos(phi1), passo2*sin(phi1)];
    end
    arr_sh = (sh_pos_new+[0, reaction_sh_norm(i)]*R1);
    line([sh_pos_new(1), arr_sh(1)], [sh_pos_new(2), arr_sh(2)], 'Color', 'r')
    hold on
    sh_pos_old = sh_pos_new;
    
end

el_pos_old = el_pos;
for i = 1:n
    
    if (i == 1)
        el_pos_new = el_pos_old;
    else
        el_pos_new = el_pos_old + [passo*cos(alpha), passo*sin(alpha)];
    end
    arr_se = (el_pos_new+[0, reaction_es_norm(i)]*R2);
    line([el_pos_new(1), arr_se(1)], [el_pos_new(2), arr_se(2)], 'Color', 'r')
    hold on
    el_pos_old = el_pos_new;
    
end
