function [paramStruct, err_mean, err_mean_perc] = minimizationProcess_4param(n_stack, L_upper, L_fore, joint, animations)

global phi desired_torque n torque err L L_2 spring y fig1 fig2 mean_err_old

if(~joint)
    L = L_fore;
    L_2 = L_upper
else
    L = L_upper;
    L_2 = L_fore;
end

n = n_stack;

%% Relevant Functions
y = @(p,phi) sqrt(L^2*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); %moment arm function
spring = @(p,phi) sqrt(L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi)); %spring length

%% Plot Setup
if(animations)
    fig1 = figure('Units','normalized');
    pos1 = get(fig1, 'Position');
    set(fig1,'Position',[.5,1-pos2(4)-.01,.5,pos2(4)+.01]); %set position of the figure window
end

%% optimization of stacks
init_param = rand(1, (n*4)); % initial guess for phi angles and radii
starting_torque = stacks_4param(init_param, phi); %torque with random guess -> calling function
mean_error_old = mean(abs(desired_torque - starting_torque)); % error with a random guess

%global variables used in 'minimize_cost.m'
%options(1) = optimset('MaxFunEvals',20000);
%options(2) = optimset('MaxIter',20000);

options.MaxIter = Inf;
options.MaxFunEvals = Inf;
%options.TolFun = 1e-6;
%options.TolX = 1e-6;

[opt_param] = minimizeCost_updateFigures4param(init_param, options, joint, animations);

paramStruct = struct('Angle',opt_param(1:n),'Radius',opt_param(n+1:2*n), 'RestLength', opt_param(2*n+1:3*n), 'Stiffness', opt_param(3*n+1;4*n));

%% Calculating Error Characteristics
err_max = max(err);
err_mean = mean(err);
err_var = var(err);
err_perc = zeros(length(err),1);

for i = 1:length(err)
    err_perc(i) = 100*err(i)/abs(desired_torque(i));
end

err_mean_perc = mean(err_perc);

%% Plot Results
if(animations)
    %adding info to error subplot
    figure(fig1)
    subplot(2,1,2)
    hold on
    plot(phi,err_mean*ones(length(phi),1),'k','linewidth',1);
    legend('Error','Mean Error','Location','Best');
    txt_1 = ['Error Variance = ', num2str(err_var)];
    txt_2 = ['Max Error = ', num2str(err_max)];
    txt_3 = ['Average Error =', num2str(err_mean)];
    %txt_4 ['Average Percentage Error =', num2str(err_mean_perc)];
    text(0,4,txt_1);
    text(0,3.5, txt_2);
    text(0,3,txt_3);
    %text(0, 2.5, txt_4);
    
    fig3 = figure('Units','normalized');
    %pos3 = get(fig3, 'Position');
    set(fig3, 'Position', [0,0,.5,1-pos1(4)-.1])
    plot(phi,torque,'b','linewidth',3);
    grid on
    hold on
    title('Desired Torque and Single Components')
    xlabel('\phi (rad)')
    ylabel('Torque (Nm)')
    plot_single_component(opt_param);
    
    fig4 = figure('Units','normalized');
    %pos4 = get(fig4, 'Position');
    set(fig4,'Position',[.5,0,.5,1-pos1(4)-.1])
    plot(phi,desired_torque,'r','linewidth',3);
    grid on
    hold on
    plot(phi,torque,'b','linewidth',3);
    hold on
    plot_single_component(opt_param);
    hold on
    axis([phi(1)-pi/12,phi(end)+pi/12,-15,15]);
    title('Desired Torque, MARIONET Torque and Single Components')
    xlabel('\phi (rad)')
    ylabel('Torque (Nm)')
    legend('Desired Torque','MARIONET Torque','Single Components','Locations','North')
end

end

