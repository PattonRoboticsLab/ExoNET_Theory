function Force_Analysis_exo(p, Exo, Bod, Torque_Exo, Torque_desired, Force_vector)
% Analizza l'andamento della forza rispetto alla lunghezza per i componenti dell'esoscheletro

nparams = Exo.nParamsSh * round(Exo.numbconstraints(2));
p_wr = p(end-2:end);
p_el = p(1:nparams+1);
p_sw = p(nparams+2:nparams*2+2);

numbsh = round( p_el(1) );    numbswivel = round( p_sw(1) );

Lrange = linspace( 0, Bod.L(1)/2 + Bod.L(2)/2, 100 );

%% Plot the torque for the shoulder for first plane of elevation (0°)
Tau_desired_sh_firstplane = vecnorm( Torque_desired.shoulder(1:2:end,:), 2, 2 );
    Tau_exo_sh_firstplane = vecnorm( Torque_Exo.shoulder(1:2:end,:), 2, 2 );

timetorque = 0:length(Tau_desired_sh_firstplane)-1;  % tempo discreto, stesso numero di campioni

figure('Name', 'Torque Comparison 0° internal rotation / 0° elbow flexion');
scatter(timetorque, vecnorm(Tau_exo_sh_firstplane, 2, 2),     'r'); hold on;
scatter(timetorque, vecnorm(Tau_desired_sh_firstplane, 2, 2), 'b');

xlabel('Frames'); ylabel('Torque (Nm)'); legend('Exoskeleton Torque', 'Desired Shoulder Torqsue'); grid on;

%% Plot the torque for the shoulder for second plane of elevation (90°)
Tau_desired_sh_secondplane = vecnorm( Torque_desired.shoulder(2:2:end,:), 2, 2 );
    Tau_exo_sh_secondplane = vecnorm(  Torque_Exo.shoulder(2:2:end,:), 2, 2 );

timetorque = 0:length(Tau_desired_sh_secondplane)-1;  % tempo discreto, stesso numero di campioni

figure('Name', 'Torque Comparison 90° internal rotation / 90° elbow flexion');
scatter(timetorque, vecnorm(Tau_exo_sh_secondplane, 2, 2),     'r'); hold on;
scatter(timetorque, vecnorm(Tau_desired_sh_secondplane, 2, 2), 'b');

xlabel('Frames'); ylabel('Torque (Nm)'); legend('Exoskeleton Torque', 'Desired Shoulder Torque'); grid on;

%% Spring elements for the elevation
figure('Name', 'Force vs Length - Elevation');
for i = 1:numbsh
    subplot(ceil(numbsh/2), 2, i); hold on; grid on;
    xlabel('Spring Length Tdist (m)'); ylabel('Force T (N)');
    title(['Elevation Spring ' num2str(i)]);

    l0 = (Bod.L(1) + p_el(2 + Exo.nParamsSh*(i-1))) / Exo.stretch_ratio_limit + ...
        ((Bod.L(1) + p_el(2 + Exo.nParamsSh*(i-1))) / Exo.stretch_ratio_limit) * p_el(5 + Exo.nParamsSh*(i-1));
    l0_recoil = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_el(9 + Exo.nParamsSh*(i-1));
    
    L_pre_extended_recoil = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_el(10 + Exo.nParamsSh*(i-1));
          
           k = p_el(7 + Exo.nParamsSh*(i-1)) * Exo.K;
    k_recoil = p_el(8 + Exo.nParamsSh*(i-1)) * Exo.K;

    force = zeros(size(Lrange));
    for j = 1:length(Lrange)
        Tdist = Lrange(j);
        T = 0;
        pre_extension = l0 - (Exo.L_rail - L_pre_extended_recoil) - Tdist;

        if pre_extension < 0
            if l0_recoil < L_pre_extended_recoil
                Tmin = k_recoil * (l0_recoil - L_pre_extended_recoil);
                Tmain = k * pre_extension;
                if Tmain < Tmin
                    k_eq = (k * k_recoil) / (k + k_recoil);
                    T = k_eq * (l0 + l0_recoil - Tdist - Exo.L_rail);
                else
                    T = Tmain;
                end
            else
                T = k * pre_extension;
            end
        end
        force(j) = T;
    end
    plot(Lrange, force);
end

%% Spring elements for the swivel
figure('Name', 'Force vs Length - Swivel');
for i = 1:numbswivel
    subplot(ceil(numbswivel/2), 2, i); hold on; grid on;
    xlabel('Spring Length Tdist (m)'); ylabel('Force T (N)');
    title(['Swivel Spring ' num2str(i)]);

    l0 = (Bod.L(1) + p_sw(2 + Exo.nParamsSh*(i-1))) / Exo.stretch_ratio_limit + ...
        ((Bod.L(1) + p_sw(2 + Exo.nParamsSh*(i-1))) / Exo.stretch_ratio_limit) * p_sw(5 + Exo.nParamsSh*(i-1));
    l0_recoil = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_sw(9 + Exo.nParamsSh*(i-1));
    
    L_pre_extended_recoil = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_sw(10 + Exo.nParamsSh*(i-1));
    
           k = p_sw(7 + Exo.nParamsSh*(i-1)) * Exo.K;
    k_recoil = p_sw(8 + Exo.nParamsSh*(i-1)) * Exo.K;

    force = zeros(size(Lrange));
    for j = 1:length(Lrange)
        Tdist = Lrange(j);
        T = 0;
        pre_extension = l0 - (Exo.L_rail - L_pre_extended_recoil) - Tdist;

        if pre_extension < 0
            if l0_recoil < L_pre_extended_recoil
                Tmin = k_recoil * (l0_recoil - L_pre_extended_recoil);
                Tmain = k * pre_extension;
                if Tmain < Tmin
                    k_eq = (k * k_recoil) / (k + k_recoil);
                    T = k_eq * (l0 + l0_recoil - Tdist - Exo.L_rail);
                else
                    T = Tmain;
                end
            else
                T = k * pre_extension;
            end
        end
        force(j) = T;
    end
    plot(Lrange, force);
end

%% Spring elements on the wrist
figure('Name', 'Force vs Length - Wrist');
xlabel('Spring Length Tdist (m)'); ylabel('Force T (N)');
title('Wrist Spring'); hold on; grid on;

l0 = (Bod.L(2) + Bod.L(1)) / Exo.stretch_ratio_limit + (Bod.L(2) + Bod.L(1)) * p_wr(1);
 k = p_wr(2) * Exo.K;

force = zeros(size(Lrange));
for j = 1:length(Lrange)
    Tdist = Lrange(j);
    pre_extension = l0 - Tdist;
    if pre_extension < 0
        force(j) = k * pre_extension;
    else
        force(j) = 0;
    end
end

plot(Lrange, force);

%% Plot the forces from the simulation
% Evaluate the norm of the force
F_des_ext = vecnorm(Force_vector.desired_extended, 2, 2);   F_exo_ext = vecnorm(Force_vector.exo_extended, 2, 2);

F_des_flex = vecnorm(Force_vector.desired_flexed, 2, 2);    F_exo_flex = vecnorm(Force_vector.exo_flexed, 2, 2);

% Sort based on desired (extended and flexed)
 [sorted_des_ext,  idx_ext] = sort(F_des_ext);    sorted_exo_ext = F_exo_ext(idx_ext);
[sorted_des_flex, idx_flex] = sort(F_des_flex);  sorted_exo_flex = F_exo_flex(idx_flex);

%% Scatter plot - Extended
figure('Name','Force Comparison - Extended');
scatter(1:length(sorted_des_ext), sorted_des_ext, 'r', 'filled'); hold on;
scatter(1:length(sorted_exo_ext), sorted_exo_ext, 'b');
legend('Desired', 'Exo');         xlabel('Sample Index (Sorted by Desired Force)');
ylabel('Force Magnitude (N)');    title('Force Magnitude Comparison - Extended');
grid on;

%% Scatter plot - Flexed
figure('Name','Force Comparison - Flexed');
scatter(1:length(sorted_des_flex), sorted_des_flex, 'r', 'filled'); hold on;
scatter(1:length(sorted_exo_flex), sorted_exo_flex, 'b');
legend('Desired', 'Exo');          xlabel('Sample Index (Sorted by Desired Force)');
ylabel('Force Magnitude (N)');     title('Force Magnitude Comparison - Flexed');
grid on;

end
