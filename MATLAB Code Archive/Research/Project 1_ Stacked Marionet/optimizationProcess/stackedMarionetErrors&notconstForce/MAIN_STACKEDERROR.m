clear all
close all
clc

global y spring desired_torque phi n L

%Load Data
%cd('C:\Users\utente\Documents\Università\UIC\Thesis\matlab\new_func_v2');
data=load('Desired_torque_fake2.txt');
phi = data(:,1);   %phi units in rad(physiological gait angle of knee -10 to 90 degrees)
desired_torque = data(:,2);   %torque desired
k=4;   %number of stacks  
L=0.26; %Units in m 26 cm
y = @(p, phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); 
spring = @(p, phi) L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi);
options.MaxIter = Inf;
options.MaxFunEvals = Inf;
options.TolFun = 1e-6;
options.TolX = 1e-6;

stacks_err = zeros(1, k);
optimal_param = zeros(k, 2*k);
for i = 1:k
    
    fprintf(' Evaluating %d Stacked Marionet \n', i);
    
    n = i;

    init_param=rand(1,(n*2)); %Initial guess for thetas and radii
    starting_torque = stacks(init_param, phi); %torque with random guess
    mean_err_old = mean(abs(desired_torque - starting_torque));  %error with random guess; global variable used in 'minimize_cost.m'
    [optimal_param(i, 1:2*i), stacks_err(i)]= minimize_cost(init_param, options, mean_err_old);
    
end

figure
plot(stacks_err, 'r');
grid on;
axis([0, n+1, 0, max(stacks_err)+(max(stacks_err)*.05)])
xlabel('N° of Stacks');
ylabel('Average Error');
