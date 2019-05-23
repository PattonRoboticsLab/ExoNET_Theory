clear all
close all
clc

global y spring desired_torque phi n L L_springRest

%Load Data
data = load('Desired_torque_multiStable.txt');
phi = data(:,1);   %angles
desired_torque = data(:,3);   %torque 
L=0.26; %Units in m 26 cm
y = @(p, phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); 
spring = @(p, phi) L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi);
L_springRest = .1;
iter = 5;
k=10;   %number of stacks  

stacks_err = zeros(1, k);
optimal_param = zeros(k, 3*k);

options.MaxIter = Inf;
options.MaxFunEvals = Inf;
options.TolFun = 1e-6;
options.TolX = 1e-6;

for i = 1:k
    
    fprintf(' Evaluating %d Stacked Marionet \n', i);
    
    n = i;
    
    [optimal_param(i, 1:i*3), stacks_err(i)] = best_parameters(iter, options);
   
end

figure
plot(stacks_err, 'r');
grid on;
axis([0, n+1, 0, max(stacks_err)+(max(stacks_err)*.05)])
xlabel('N° of Stacks');
ylabel('Average Error [Nm]');
title('Average Error VS N° of Stacks');
