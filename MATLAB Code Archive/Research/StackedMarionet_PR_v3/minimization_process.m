function [optimal_parameter,error_mean] = minimization_process(n_stacks, L_upper, L_fore, joint, animations)
global phi desired_torque n torque error L L2 Lsp mom_arm fig1 fig2 mean_init_error Lsp_rest


%% Assign Upper Arm Length and Forearm length based on Device Location
if(~joint)
    L = L_fore;
    L2 = L_upper;
else
    L = L_upper;
    L2 = L_fore;
end


n = n_stacks; %number of MARIONET stacks

%% Main System Function Definitions
mom_arm = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); %Moment Arm Formula: p(1) - Peg Angle, p(2) - Peg Radius
Lsp = @(p,phi) sqrt(L^2 + p(2)^2-2*L*p(2).*cos(p(1)-phi)); %spring length as a function of Peg Angle and Radius

%% Plot Setup
if (animations)
    fig1 = figure('Units','normalized');
    pos1 = get(fig1,'Position');
    set(fig1, 'Position'); %set position of the figure window
    
    %Marionet
    fig2 = figure('Units','normalized');
    pos2 = get(fig2, 'Position');
    set(fig2, 'Position'); %set position of the figure window
end

%% Stacked Element Optimization
initial_parameters = rand(1,(n*2)); %initial guesses for the Peg Angles and Radii
starting_torque = stacks(initial_parameters,phi) %Starting Torque Value based on Initial Random Guess


error = abs(desired_torque-starting_torque); % error profile of the starting,random torque value and desired torque profile
mean_init_error = mean(error); %mean of the initial error

% Setting the Options for fminsearch optimization

%options(1) = optimset('MaxFunEvals',20000);
%options(2) = optimset('MaxIter',20000);

options.MaxIter = Inf;
options.MaxFunEvals = Inf;
options.Tolfun = 1e-6;
options.TolX = 1e-6;

%% Calling Cost Function and Figure Updates
[optimal_parameter, errorVector] = minimizeCost_updateFigures(initial_parameters, options, joint, animations);

%% Calculating Error Characteristics
error_max = max(error)
error_mean = mean(error)
error_var = var(error)

error_percentile = zeros(length(error),1);
for i = 1:length(error)
    error_percentile(i) = 100*error(i)/abs(desired_torque(i));
end

error_percentile_mean = mean(error_percentile)
SS_total = sum((desired_torque - mean(desired_torque)).^2);
SS_res = sum(error.^2);
Rsquared = 1-(SS_res/SS_total);


%% Plot Results
if(animations) %adding information to the error subplots
    %Figure 1
    figure(fig1)
    subplot(2,1,2)
    hold on
    plot(phi,error_mean*ones(length(phi),1),'k','linewidth',1);
    legend('Error','Mean Error','Location','Best');
    
    txt1 = ['Error Variance =', num2str(error_var)];
    text(0,4,txt1);
    
    txt2 = ['Maximum Error =',num2str(error_max)];
    text(0,3.5,txt2)
    
    txt3 = ['Average Error =',num2str(error_mean)];
    text(0,3,txt3);
    
    txt4 = ['Average Percentage Error=',num2str(error_percentile_mean)];
    text(0,2.5,txt4)
    
    txt5 = ['R Squared = ',num2str(Rsquared)];
    text(0,2,txt5)
    
    %figure 3
    figure(fig3)
    figure('Unit','normalized');
    set(fig3,'Position')
    plot(phi,torque,'b','linewidth',3);
    title('Desired Torque and Single Components')
    xlabel('\phi (Radians)')
    ylabel('Torque (Nm)')
    
    grid on
    hold on
    
    plot_single_component(optimal_parameter);
    
    %figure 4
    figure(fig4)
    figure('Units','normalized') %pos4 = get(fig4,'Position');
    set(fig4,'Position')
    plot(phi,desired_torque,'r','linewidth',3);
    
    grid on
    hold on
    
    plot(phi,torque,'b','linewidth',3);
    hold on
    plot_single_component(optimal_paramater)
    hold on
    axis([phi(1)-pi/12,phi(end)+pi/12,-15,15]);
    title('Desired Torque','MARIONET TOrque','Single COmponent','Location','North');
    
    %Figure 5
    figure(fig5)
    figure('Units','normalized');
    plot(errorVector,'r');
    grid on
    xlabel('Number of Iterations')
    ylabel('Average Absolute Error [Nm]')
    ylim([0,max(errorVector)+.1]);
    xlim([0,length(errorVector)+1]);
    title('Average Absolute Error vs While Loop Iterations')
end

    
    
    
    
    




   









