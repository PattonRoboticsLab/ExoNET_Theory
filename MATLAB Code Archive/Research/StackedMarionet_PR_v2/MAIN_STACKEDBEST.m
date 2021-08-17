clear all
close all
clc

global y spring desired_torque phi n L L_springRest springStiffness

%Load Data
data = load('Desired_torque_profile.txt');
phi = data(:,1);   %phi units in rad(physiological gait angle of knee -10 to 90 degrees)
desired_torque = data(:,2);   %torque 

%% Given Known Variables
L=0.26; %Units in m 26 cm
iter = 2; %number of iterations
k=5;   %number of stacks  
L_springRest = .1;
springStiffness = 500;

%% Functions 
y = @(p, phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); 
spring = @(p, phi) sqrt(L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi));


stacks_err = zeros(1, k);
average_perc = zeros(1, k);
optimal_param = zeros(k, 2*k);
optimal_param_perc = zeros(k, 2*k);

%% Options for fminsearch Optimizations
options.MaxIter = Inf;
options.MaxFunEvals = Inf;
options.TolFun = 1e-6;
options.TolX = 1e-6;

for i = 1:k
    
    fprintf(' Evaluating %d Stacked Marionet \n', i);
    
    n = i;
    
    [optimal_param(i, 1:i*2), optimal_param_perc(i, 1:i*2), stacks_err(i), average_perc(i)] = best_parameters(iter, options);
   
end


figure
plot(stacks_err, 'r');
grid on;
axis([0, n+1, 0, max(stacks_err)+(max(stacks_err)*.05)])
xlabel('N° of Stacks');
ylabel('Average Error [Nm]');
title('Average Error VS N° of Stacks');
figure
plot(average_perc, 'r');
grid on;
axis([0, n+1, 0, max(average_perc)+(max(average_perc)*.05)])
xlabel('N° of Stacks');
ylabel('Average Percentage Error [%]');
title('Average Error VS N° of Stacks');

figure
plot(phi,desired_torque,'r','linewidth',3);
grid on
hold on
plot(phi,stacks(optimal_param,phi));
hold on
plot_single_component(optimal_param);
hold on
title('Desired Torque, MARIONET Torque and Single Components')
xlabel('\phi (rad)')
ylabel('Torque (Nm)')
legend('Desired Torque','MARIONET Torque','Single Components','Location','North')
