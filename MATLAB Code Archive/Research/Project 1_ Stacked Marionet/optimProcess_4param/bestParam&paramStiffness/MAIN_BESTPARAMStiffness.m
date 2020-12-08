clear all
close all
clc

global phi L desired_torque n y spring L_springRest

data=load('Desired_torque_multiStable.txt');
phi = data(:,1);   %phi units in rad(physiological gait angle of knee -10 to 90 degrees)
desired_torque = data(:,3);   %torque desired
n=4;   %number of stacks  
L=0.26; %Units in m 26 cm
%options = optimset('MaxFunEvals',10000);
y = @(p, phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)));
spring = @(p, phi) L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi);
L_springRest = .10;
iter = 50;
parameters = zeros(iter, 3*n);
mean_err = zeros(1, iter);
cost = zeros(1, iter);

options.MaxIter = Inf;
options.MaxFunEvals = Inf;


for i = 1:iter
    
    fprintf(' Evaluation number: %d \n', i);
    init_param=rand(1,(n*3)); %Initial guess for thetas and radii
    err = abs(desired_torque - stacks(init_param, phi));
    mean_err_old = mean(err); 
    [parameters(i, :), average_err] = minimize_cost(init_param, options, mean_err_old);
    mean_err(i) = average_err;
    
    if i == 1
        min_mean = average_err;
        max_mean = average_err;
    end
    
    if average_err <= min_mean
        min_mean = average_err;
        pos_min = i;
    elseif average_err >= max_mean
        max_mean = average_err;
        pos_max = i;
    end

end

figure(1)
plot(mean_err, 'r');
axis([0, iter+1, 0, max(mean_err)+max(mean_err)*.05]);
hold on
grid on
plot(pos_min, min_mean, 'go', 'linewidth', 2);
hold on
plot(pos_max, max_mean, 'bo', 'linewidth', 2);
legend('Average Error', 'Minimum Avg Error', 'Max Avg Error', 'Location', 'Best');
xlabel('#Iteration');
ylabel('Average Error [Nm]');
plot_title1 = ['Average Error During #Iterations: ', num2str(iter)];
title(plot_title1)

figure(2)
subplot(2, 1, 1)
plot(phi, desired_torque, 'r', 'linewidth', 2);
hold on
grid on
best_par1 = parameters(pos_min, :);
tau_tot1=stacks(best_par1, phi);
plot(phi, tau_tot1, 'b', 'linewidth', 4);
hold on
legend('Desired Profile', 'Best Profile', 'Location', 'Best');
xlabel('\Phi(Radians)');
ylabel('Torque [Nm]');
plot_title2 = ['Torque Profile with Best Parameters after #Iterations: ', num2str(iter)];
title(plot_title2)

subplot(2, 1, 2)
real_err1 = abs(desired_torque - tau_tot1);
plot(phi, real_err1, 'g', 'linewidth', 2);
hold on
grid on
xlabel('\Phi(Radians)');
ylabel('Error [Nm]');
txt_1 = ['Error = ', num2str(mean(real_err1))];
text(.5, 2, txt_1);
plot_title3 = ['Error with Best Parameters after #Iterations: ', num2str(iter)];
axis([phi(1)-pi/12, phi(end)+pi/12, -0.25, 5]);
title(plot_title3)

figure(3)
subplot(2, 1, 1)
plot(phi, desired_torque, 'r', 'linewidth', 2);
hold on
grid on
best_par2 = parameters(pos_max, :);
tau_tot2=stacks(best_par2, phi);
plot(phi, tau_tot2, 'b', 'linewidth', 4);
hold on
legend('Desired Profile', 'Worst Profile', 'Location', 'Best');
xlabel('\Phi(Radians)');
ylabel('Torque [Nm]');
plot_title4 = ['Torque Profile with Worst Parameters after #Iterations: ', num2str(iter)];
title(plot_title4)

subplot(2, 1, 2)
real_err2 = abs(desired_torque - tau_tot2);
plot(phi, real_err2, 'g', 'linewidth', 2);
hold on
grid on
xlabel('\Phi(Radians)');
ylabel('Error [Nm]');
txt_2 = ['Error = ', num2str(mean(real_err2))];
text(.5, 2, txt_2);
plot_title5 = ['Error with Worst Parameters after #Iterations: ', num2str(iter)];
axis([phi(1)-pi/12, phi(end)+pi/12, -0.25, 5]);
title(plot_title5)

