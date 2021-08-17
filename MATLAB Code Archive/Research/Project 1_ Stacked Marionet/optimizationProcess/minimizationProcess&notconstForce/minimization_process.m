function [opt_param, avg_error] = minimization_process(n_stackedMarionet, L_upperArm, L_foreArm, choosingJoint, animations)

    global phi desired_torque n torque err L L_2 spring y fig1 fig2 mean_err_old
    
    if (~choosingJoint)
        L = L_foreArm; %Units in m 26 cm
        L_2 = L_upperArm;
    else
        L = L_upperArm;
        L_2 = L_foreArm;
    end

    n = n_stackedMarionet;
    
    %functions
    y = @(p, phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); %arm for the force 
    spring = @(p, phi) sqrt(L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi)); %spring length

    %% Plot setup
    if (animations)
        fig1 = figure('Units', 'normalized');
        pos1 = get(fig1, 'Position');
        set(fig1, 'Position', [0, 1-pos1(4)-.01, .5, pos1(4)+.01]) %set position of the figure window

        %Marionet
        fig2 = figure('Units', 'normalized');
        pos2 = get(fig2, 'Position');
        set(fig2, 'Position', [.5, 1-pos2(4)-.01, .5, pos2(4)+.01]); %set position of the figure window
    end
    %% optimization of stacks

    init_param=rand(1,(n*2)); %Initial guess for thetas and radii
    starting_torque = stacks(init_param, phi); %torque with random guess
    mean_err_old = mean(abs(desired_torque - starting_torque));  %error with random guess; global variable used in 'minimize_cost.m'
    %options(1) = optimset('MaxFunEvals',20000);
    %options(2) = optimset('MaxIter',20000);
    options.MaxIter = Inf;
    options.MaxFunEvals = Inf;
    options.TolFun = 1e-6;
    options.TolX = 1e-6;
    [opt_param, errVec] = minimizeCost_updateFigures(init_param, options, choosingJoint, animations); 

    %% Calculating error characteristics

    err_max = max(err);
    err_mean = mean(err);
    err_var = var(err);
    err_perc = zeros(length(err), 1);
    for i = 1:length(err)
        err_perc(i) = 100*err(i)/abs(desired_torque(i));
    end
    err_mean_perc = mean(err_perc);
    SS_tot = sum((desired_torque-mean(desired_torque)).^2);
    SS_res = sum(err.^2);
    Rsquared = 1 - (SS_res/SS_tot);
    %% Plot results
    if (animations)
        %Adding info to Error subplot
        figure(fig1)
        subplot(2, 1, 2)
        hold on
        plot(phi, err_mean*ones(length(phi), 1), 'k', 'linewidth', 1);
        legend('Error', 'Mean Error', 'Location', 'Best');
        txt_1 = ['Error Variance = ', num2str(err_var)];
        txt_2 = ['Max Error = ', num2str(err_max)];
        txt_3 = ['Average Error = ', num2str(err_mean)];
        txt_4 = ['Average Percentage Error = ', num2str(err_mean_perc)];
        txt_5 = ['R^2 = ', num2str(Rsquared)];
        text(0, 4, txt_1);
        text(0, 3.5, txt_2);
        text(0, 3, txt_3);
        text(0, 2.5, txt_5);

        fig3 = figure('Units', 'normalized');
        %pos3 = get(fig3, 'Position');
        set(fig3, 'Position', [0, 0, .5, 1-pos1(4)-.1])
        plot(phi, torque, 'b', 'linewidth', 3);
        grid on
        hold on
        title('Desired Torque and Single Components')
        xlabel('\phi (rad)')
        ylabel('Torque (Nm)')
        plot_single_component(opt_param);

        fig4 = figure('Units', 'normalized');
        %pos4 = get(fig4, 'Position');
        set(fig4, 'Position', [.5, 0, .5, 1-pos1(4)-.1])
        plot(phi, desired_torque, 'r', 'linewidth', 3);
        grid on
        hold on
        plot(phi, torque, 'b', 'linewidth', 3);
        hold on
        plot_single_component(opt_param);
        hold on
        axis([phi(1)-pi/12, phi(end)+pi/12, -15, 15]);
        title('Desired Torque, MARIONET Torque and Single Components')
        xlabel('\phi (rad)')
        ylabel('Torque (Nm)')
        legend('Desired Torque', 'MARIONET Torque', 'Single Components', 'Location', 'North')
        
        fig5 = figure('Units', 'normalized');
        plot(errVec, 'r');
        grid on
        xlabel('# of Iterations')
        ylabel('Average Abs Error [Nm]')
        ylim([0, max(errVec)+.1])
        xlim([0, length(errVec)+1])
        title('Average Abs Error VS While Loop Iterations')
    end
end