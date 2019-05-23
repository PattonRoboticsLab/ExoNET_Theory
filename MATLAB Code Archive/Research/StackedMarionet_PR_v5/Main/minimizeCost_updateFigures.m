function [p_new, errVec] = minimizeCost_updateFigures(p, options, choosingJoint, animations)

    %% Function objective
    %this function minimizes the cost given by the 'torque_cost.m' function and starts the animations.
    %The minimization process stops when the difference between average error at the current and the one at the previous 
    %iteration is smaller than a certain tollerance.
    
    global R phi h torque desired_torque err g fig1 fig2 n mean_err_old L_2 L 
    
    toll = 1e-5; %stop tollerance
    max_iter = 100;
    it = 1;  
    %% First plots
    
    %First iteration
    p_new = fminsearch('torque_cost', p, options);
    torque = stacks(p_new, phi);
    err = abs(desired_torque - torque);

    %Torque subplot
    if (animations)
        figure(fig1);
        subplot(2, 1, 1);
        h = plot(phi, torque, 'b', 'linewidth', 3);      
        hold on
        plot(phi, desired_torque, 'r', 'linewidth', 3); 
        hold on
        legend('MARIONET','Desired', 'Location', 'Best');
        grid on
        axis([phi(1)-pi/12, phi(end)+pi/12, -5, 10]); %0, 15
        title('Torque profile');
        xlabel('\bf\Phi(Radians)')
        ylabel('Torque (Nm)')


        %Error subplot
        subplot(2, 1, 2);
        g = plot(phi, err, 'g', 'linewidth', 2);
        grid on
        %g.YDataSource = 'err'; %same as the previous one
        axis([phi(1)-pi/12, phi(end)+pi/12, -0.25, 5]);
        title('Error');
        xlabel('\bf\Phi(Radians)')
        ylabel('Error (Nm)')

        %Building marionet figures

        radius = p_new(1, n+1:end);
        angle = p_new(1, 1:n);
        shoulder_angle = -pi/4;
        elbow_angle = pi/6;
        alpha = elbow_angle+shoulder_angle;
        R_el = [cos(shoulder_angle), -sin(shoulder_angle); sin(shoulder_angle), cos(shoulder_angle)];
        R_wr = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];
        sh_pos = [0; 0]; %shoulder position
        
        if (~choosingJoint)
            el_pos = sh_pos+(R_el*[L_2; 0]); %elbow position
            wr_pos = el_pos+(R_wr*[L; 0]);   %wrist position
            starting_point = el_pos;
            ending_point = wr_pos;
        else
            el_pos = sh_pos+(R_el*[L; 0]); %elbow position
            wr_pos = el_pos+(R_wr*[L_2; 0]);   %wrist position
            starting_point = sh_pos;
            ending_point = el_pos;
        end

        draw_system(sh_pos, el_pos, wr_pos);
        hold on

        %Drawing the marionets and taking the handles of the figures
        handles = zeros(n, 3); %n differetn marionets, and each has 3 handles

        for i = 1:n
            f = draw_marionet_handles(radius(i), angle(i), starting_point, ending_point);
            handles(i, :) = f;
        end

        axis([-.15, .65, -.7, .25])
        title('Marionet Scheme')
        drawnow
        
    end
    %% Minimization process
    mean_err_new = mean(err);
    err_diff = abs(mean_err_new-mean_err_old);
    errVec = [];
    
    while (((err_diff > toll) || (mean_err_new > mean_err_old)) && (it < max_iter)) %when the difference between the new and the old average error 
                                                                %decreases under a cetain threshold, we exit the while loop.  
        mean_err_old = mean_err_new;
        p_old = p_new;
        p_new = fminsearch('torque_cost', p_old, options);
        torque = stacks(p_new, phi);
        err = abs(desired_torque - torque);
        mean_err_new = mean(err);
        err_diff = abs(mean_err_new-mean_err_old);
        errVec = [errVec, mean_err_new];
        if (animations)
            %Refreshing plots
            figure(fig1)
            set(h, 'YData', torque);
            set(g, 'YData', err)

            figure(fig2)
            radius = p_new(1, n+1:end)
            angle = p_new(1, 1:n)
            R = table(radius,angle)
            splength = @(p, phi) sqrt(L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi)); %spring length
            %drawing each marionet
            

            for i=1:n
                draw_marionet_update(radius(i), angle(i), starting_point, ending_point, handles(i, :));
                
            end
            drawnow
            
        end
    end

end
