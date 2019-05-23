clear all
close all
clc

global stack tau_d phi n L

%Load Data
%cd('C:\Users\utente\Documents\Università\UIC\Thesis\matlab\new_func_v2');
data=load('Desired_torque_fake2.txt');
phi=data(:,1);   %phi units in rad(physiological gait angle of knee -10 to 90 degrees)
tau_d=data(:,2);   %torque desired
k=7;   %number of stacks  
L=0.26; %Units in m 26 cm
stacks_err = zeros(1, k);

for i = 1:k
    
    fprintf(' Evaluating %d Stacked Marionet \n', i);
    
    n = i;
    create_stack;

    init_param=rand(1,(n*2)); %Initial guess for thetas and radii
    options = optimset('MaxFunEvals',10000);
    opt_param = min_cost_err(init_param, options);
    
    torque = stack(opt_param, phi);
    err = abs(tau_d - torque);
    err_mean = mean(err);
    stacks_err(i) = err_mean;    
end

figure
plot(stacks_err, 'r');
grid on;
xlabel('N° of Stacks');
ylabel('Mean Error');
