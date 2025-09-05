function Force_vector = create_plots(Exo, Actual_Pin, bestP_3D, L0_recoil, TauExo, TAUsDesired, q, Bod, Pos, robot)
% Create all the visualization plots
fprintf('Creating comprehensive visualization...\n');

% Define body positions
shoulder_pin = Exo.shoulder;                       
center_back = [0, -0.146, 1.15];
shoulder_el = [0.169316, -0.0167571, 1.39712];    
shoulder_sw = [0.195279 -0.0285629 1.37362];
elbow_el = [0.247853   0.221341  1.38841];        
elbow_sw = [0.280967 0.207069 1.36473];
wrist = [0.357447, 0.458538, 1.36375];         

shoulder_el_flex = [0.141, 0.0382423, 1.30084];   
shoulder_sw_flex = [0.185, -0.00145449, 1.37209];
elbow_el_flex = [0.153,   0.172,   1.23023];      
elbow_sw_flex = [0.182581 0.186 1.25767];
wrist_flex = [-0.0652559, 0.195, 1.29763];         

% Figure 1: Elbow extended configuration
figure('Name', 'Error field elbow extended', 'Position', [50, 50, 1200, 800]); 
nancy_body; 
hold on; 
title('0° internal rotation / 0° elbow flexion', 'FontSize', 14);
legend4 = PlotPin(Exo, Actual_Pin, shoulder_pin, shoulder_el, shoulder_sw, elbow_sw, elbow_el, bestP_3D, center_back, wrist, L0_recoil);
Force_vector.exo_extended = plotVectField3D(q, Bod, Pos, Exo, robot, TauExo.elevationSh, TauExo.elevationEl, 'b', 3, 4, 2);
Force_vector.desired_extended = plotVectField3D(q, Bod, Pos, Exo, robot, TAUsDesired.TauSh_tot, TAUsDesired.TauEl_tot, 'r', 3, 4, 2);
grid on; 
legend('Exoskeleton Forces', 'Desired Forces', 'Location', 'best');
hold off;

% Figure 2: Elbow flexed configuration
figure('Name', 'Error field elbow flexed', 'Position', [100, 100, 1200, 800]); 
nancy_body_flex; 
hold on; 
title('90° internal rotation / 90° elbow flexion', 'FontSize', 14);
legend5 = PlotPin(Exo, Actual_Pin, shoulder_pin, shoulder_el_flex, shoulder_sw_flex, elbow_sw_flex, elbow_el_flex, bestP_3D, center_back, wrist_flex, L0_recoil);
Force_vector.exo_flexed = plotVectField3D(q, Bod, Pos, Exo, robot, TauExo.elevationSh, TauExo.elevationEl, 'b', 3, 4, 1);
Force_vector.desired_flexed = plotVectField3D(q, Bod, Pos, Exo, robot, TAUsDesired.TauSh_tot, TAUsDesired.TauEl_tot, 'r', 3, 4, 1);
grid on; 
legend('Exoskeleton Forces', 'Desired Forces', 'Location', 'best');
hold off;

% Figure 3: Pin configuration only
figure('Name', 'Pin Configuration', 'Position', [150, 150, 800, 600]); 
hold on; 
title('Optimized Pin Configuration', 'FontSize', 14);
legend6 = PlotPin(Exo, Actual_Pin, shoulder_pin, shoulder_el, shoulder_sw, elbow_sw, elbow_el, bestP_3D, center_back, wrist, L0_recoil);
grid on;
hold off;

% Figure 4: Torque comparison - First plane
create_torque_comparison_plot(TAUsDesired, TauExo, 1, '0° internal rotation / 0° elbow flexion');

% Figure 5: Torque comparison - Second plane  
create_torque_comparison_plot(TAUsDesired, TauExo, 2, '90° internal rotation / 90° elbow flexion');

fprintf('All plots created successfully!\n');
end

function create_torque_comparison_plot(TAUsDesired, TauExo, plane, title_str)
% Create torque comparison plots
if plane == 1
    Tau_desired_sh = vecnorm(TAUsDesired.TauSh_tot(1:2:end, :), 2, 2);
    Tau_exo_sh = vecnorm(TauExo.elevationSh(1:2:end, :), 2, 2);
else
    Tau_desired_sh = vecnorm(TAUsDesired.TauSh_tot(2:2:end, :), 2, 2);
    Tau_exo_sh = vecnorm(TauExo.elevationSh(2:2:end, :), 2, 2);
end

timetorque = 0:length(Tau_desired_sh)-1;

figure('Name', sprintf('Torque Comparison - Plane %d', plane), 'Position', [200+50*plane, 200+50*plane, 1000, 600]);
subplot(2, 1, 1);
plot(timetorque, Tau_exo_sh, 'r-', 'LineWidth', 2, 'DisplayName', 'Exoskeleton Torque');
hold on;
plot(timetorque, Tau_desired_sh, 'b-', 'LineWidth', 2, 'DisplayName', 'Desired Torque');
title(title_str, 'FontSize', 14);
xlabel('Frames');
ylabel('Torque (Nm)');
legend('show', 'Location', 'best');
grid on;

subplot(2, 1, 2);
error_torque = abs(Tau_exo_sh - Tau_desired_sh);
plot(timetorque, error_torque, 'k-', 'LineWidth', 2);
title('Torque Error', 'FontSize', 12);
xlabel('Frames');
ylabel('Absolute Error (Nm)');
grid on;

% Add statistics
mean_error = mean(error_torque);
max_error = max(error_torque);
rms_error = sqrt(mean(error_torque.^2));

text(0.7, 0.9, sprintf('Mean Error: %.3f Nm', mean_error), 'Units', 'normalized', 'FontSize', 10);
text(0.7, 0.85, sprintf('Max Error: %.3f Nm', max_error), 'Units', 'normalized', 'FontSize', 10);
text(0.7, 0.8, sprintf('RMS Error: %.3f Nm', rms_error), 'Units', 'normalized', 'FontSize', 10);
end

function save_optimization_history(collect_p, bestCost, optimization_results)
% Save optimization history to file
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('optimization_history_%s.mat', timestamp);

optimization_data = struct();
optimization_data.best_parameters = collect_p;
optimization_data.best_cost = bestCost;
optimization_data.timestamp = timestamp;
optimization_data.all_results = optimization_results;

save(filename, 'optimization_data');
fprintf('Optimization history saved to: %s\n', filename);
end