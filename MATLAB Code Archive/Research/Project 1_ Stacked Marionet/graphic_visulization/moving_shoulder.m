clear all
close all
clc

n = 50;
L_upper = 0.35;
L_fore = 0.26;

g = 9.81;   %gravity constant
m_tot = 70; %total body mass in Kg
m_upper = (2.71/100)*m_tot; %mass uppòer ar
m_fore = (1.62/100)*m_tot;  %mass forearm
m_hand = (0.61/100)*m_tot;  %mass hand

wh = m_hand*g; %weight of the hand
limit_min1 = 0;
limit_max1 = L_fore;
limit_min2 = 0;
limit_max2 = L_upper;
armel = linspace(limit_min1, limit_max1, n);
passo = limit_max1/(n-1);
armsh = linspace(limit_min2, limit_max2, n);
passo2 = limit_max2/(n-1);

fun_half2_es = @(x2, phi1, phi2) -wh*(x2*cos(phi2)+L_upper*cos(phi1));
fun_half1_es = @(x2, phi1, phi2) -wh*(2*x2*cos(phi2)+L_upper*cos(phi1))-m_fore*g*(L_upper*cos(phi1)+x2*cos(phi2));
armse_1 = linspace(limit_min1, limit_max1, n);

fun_half1_sh = @(x, phi1) -wh*(x*cos(phi1))-m_fore*g*(x*cos(phi1));
fun_half2_sh = @(x, phi1) -m_upper*g*x*cos(phi1)-wh*(x*cos(phi1))-m_fore*g*(x*cos(phi1));

k = 10;
phi = linspace(-pi/2, pi/2, k);
phi2 = linspace(0, 2*pi/3, k);

sh_pos = [0; 0];
head_pos = [0; .15];
bas_pos = [0; -.35];
dist_headbas = [head_pos, bas_pos];
nose_pos = [.07; .1];
dist_noshead = [head_pos, nose_pos];

figure(1)

plot(dist_headbas(1, :), dist_headbas(2, :), 'g', 'linewidth', 5);
grid on
hold on
scatter(head_pos(1), head_pos(2), 2000, 'k', 'filled');
hold on
scatter(sh_pos(1), sh_pos(2), 100, 'k', 'filled');
hold on
plot(dist_noshead(1, :), dist_noshead(2, :), 'k', 'linewidth', 5);

for iter = 1:k
    for j = 1:k
        
        phi1 = phi(iter);
        alpha = phi2(j)+phi1;
        R1 = [cos(phi1), -sin(phi1); sin(phi1), cos(phi1)];
        R2 = [cos(+alpha), -sin(+alpha); sin(+alpha), cos(+alpha)];
        el_pos = sh_pos + [L_upper*cos(phi1); L_upper*(sin(phi1))];
        dist_se = [sh_pos, el_pos];
        sh_el = [(sh_pos(1)+el_pos(1))/2; (sh_pos(2)+el_pos(2))/2];
        wr_pos = el_pos + [L_fore*cos(alpha); L_fore*(sin(alpha))];
        el_wr = [(el_pos(1)+wr_pos(1))/2; (el_pos(2)+wr_pos(2))/2];
        dist_ew = [el_pos, wr_pos];
        
        a = plot(dist_ew(1, :), dist_ew(2, :), 'g', 'linewidth', 5);
        hold on
        b = scatter(wr_pos(1), wr_pos(2), 100, 'k', 'filled');
        hold on
        c = scatter(el_wr(1), el_wr(2), 'k', 'filled');
        hold on
        pt_elb = el_wr-[0; .2*m_fore];
        d = arrow(el_wr, pt_elb);
        hold on
        pt_wrist = wr_pos-[0; .2*m_hand];
        e = arrow(wr_pos, pt_wrist);
        hold on
        f = plot(dist_se(1, :), dist_se(2, :), 'g', 'linewidth', 5);
        hold on
        s = scatter(el_pos(1), el_pos(2), 100, 'k', 'filled');
        hold on
        g = scatter(sh_el(1), sh_el(2), 'k', 'filled');
        hold on
        pt_sh = sh_el-[0; .2*m_upper];
        h = arrow(sh_el, pt_sh);      
        
        reaction_es = zeros(1, n);
        for i = 1:n

            if (armse_1(i)<=L_fore/2)
                reaction_es(i) = fun_half1_es(armse_1(i), phi1, phi2(j));  
            elseif (armel(i)>L_fore/2)
                reaction_es(i) = fun_half2_es(armse_1(i), phi1, phi2(j));
            end

        end

        reaction_sh = zeros(1, n);
        for i = 1:n

            if (armsh(i)<=L_upper/2)
                reaction_sh(i) = fun_half2_sh(armsh(i), phi1);  
            elseif (armsh(i)>L_upper/2)
                reaction_sh(i) = fun_half1_sh(armsh(i), phi1);
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
        handles = [];
        sh_pos_old = sh_pos;
        for i = 1:n

            if (i == 1)
                sh_pos_new = sh_pos_old;
            else
                sh_pos_new = sh_pos_old + [passo2*cos(phi1); passo2*sin(phi1)];
            end
            arr_sh = (sh_pos_new+R1*[0; reaction_sh_norm(i)]);
            l1 = line([sh_pos_new(1), arr_sh(1)], [sh_pos_new(2), arr_sh(2)], 'Color', 'r');
            handles = [handles, l1];
            hold on
            sh_pos_old = sh_pos_new;

        end

        el_pos_old = el_pos;
        for i = 1:n

            if (i == 1)
                el_pos_new = el_pos_old;
            else
                el_pos_new = el_pos_old + [passo*cos(alpha); passo*sin(alpha)];
            end
            
            if (atan2(wr_pos(2)-el_pos(2), wr_pos(1)-el_pos(1)) >= pi/2)
                arr_se = (el_pos_new+R2*[0; -reaction_es_norm(i)]);
            else
                arr_se = (el_pos_new+R2*[0; reaction_es_norm(i)]);
            end
            l2 = line([el_pos_new(1), arr_se(1)], [el_pos_new(2), arr_se(2)], 'Color', 'r');
            hold on
            handles = [handles, l2];
            el_pos_old = el_pos_new;

        end

        axis([-.25 .65, -0.65, .65])
        drawnow
        if (iter ~= k || j ~= k)
            delete(a)
            delete(b)
            delete(c)
            delete(e)
            delete(d)
            delete(f)
            delete(g)
            delete(h)
            delete(s)
            delete(handles)
        end
    end
end