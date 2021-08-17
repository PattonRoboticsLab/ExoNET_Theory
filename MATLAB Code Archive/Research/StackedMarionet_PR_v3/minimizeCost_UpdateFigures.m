function [optimal_parameter, errorVector] = minimizeCost_UpdateFigures(p, options, joint, animations)

%%Function Objective: This Function minimizes the cost given by the
%%'torque_cost.m' function and starts the animations. The minmization
%%process stops when the difference btween average error at the current
%%and the one at the previous iteration is smaller than a certain tolerance

global phi h torque desired_torque error g fig1 fig2 n mean_init_error

toll = 1e-5; %setting the tolerance value
max_iteration = 100; %setting the maximum number of iterations for process
it = 1;

%% First Plots

%first iteration
param_new = fminsearch('torque_cost',p,options);
torque = stacks(param_new, phi);

% error = abs(desired_torque - torque) % already coded in
% minimization.process, is this necessary to code again??



if(animations)
    figure(fig1); 
    subplot(2,1,1); %torque subplot
    h = plot(phi,torque,'b','linewidth','3');
    hold on
    plot(phi,desired_torque,'r','linewidth',3);
    hold on
    legend('MARIONET','Desired','Location','Best');
    grid on
    axis([phi(1)-pi/12,phi(end)+pi/12,-5,10]);
    title('Torque Profile');
    xlabel('\bf \Phi(radians)')
    ylabel('Torque (Nm)')
    
    subplot (2,1,2) %Error Subplot
    g = plot(phi, error, 'g','linewidth',2);
    grid on
    %g.YDataSource = 'error'; %same as previous one
    axis([phi(1)-pi/12;phi(end)+pi/12,-0.25,5]);
    title('Error');
    xlabel('\bf \Phi(Radians)')
    ylabel('Error (Nm)')
    
%Building MARIONET Figures
radius = param_new(1,n+1:end);
angle = param_new(1,1:n);

shoulder_angle = -pi/4;
elbow_angle = pi/6;
alpha = elbow_angle + shoulder_angle;
R_elbow = [cos(shoulder_angle),-sin(shoulder_angle),sin(shoulder_angle),cos(shoulder_angle)];
R_wrist = [cos(alpha),-sin(alpha),sin(alpha),cos(alpha)];

shoulder_pos = [0;0]; %shoulder position

if(joint)
    elbow_pos = shoulder_pos+(R_elbow *[L2;0]); %elbow position
    wrist_pos = elbow_pos+(R_wrist*[L;0]); %wrist position
    starting_point = elbow_pos;
    ending_point = wrist_pos;
else
    elbow_pos = shoulder_pos + (R_elbow*[L;0]); %elbow position
    wrist_pos = elbow_pos + (R_wrist *[L2;0]); %wrist position
    starting_point = shoulder_position;
    ending_point = elbow_position;
end

%% Calling Draw System Function
draw_system(shoulder_pos,elbow_pos,wrist_pos);
hold on

%Drawing MARIONETs and Taking Handles of Figures
handles = zeros(n,3); %n different MARIONETs, each with 3 handles

for i=1:n
    f = draw_marionet_handles(radius(i),angle(i),starting_point,ending_point);
    handles(i,:) = f;
end

axis([-0.15,0.65, -0.7, 0.25])
title('Marionet Scheme')
drawnow
end

%% Minimization Process
mean_new_error = mean(error);
diff_error = abs(mean_new_error-mean_init_error)
errVector = [];

while (((diff_error > toll) || (mean_new_error > mean_init_error)) && (it < max_iter))
    %when the difference between the new and old average error decreases
    %under a certain threshold, we exit the while loop
    mean_init_error = mean_new_error;
    param_old = param_new;
    param_new = fminsearch('torque_cost',param_old,options);
    torque = stacks(param_new,phi);
    error = abs(desired_torque - torque);
    mean_new_error = mean(error);
    diff_error = abs(mean_new_error - mean_init_error);
    errVector = [errVector, mean_new_error];

    if(animations)
        %refreshing plots
    figure(fig1)
    set(h,'YData',torque);
    set(g,'YData',error);
    
    figure(fig2)
    radius = param_new(1,n+1:end);
    angle = param_new(1,1:n);
    
    %drawing with each MARIONET
for i = 1:n
    draw_marionet_update(radius(i),angle(i),starting_point,ending_point,handles(i,:));
end
drawnow
end
end

