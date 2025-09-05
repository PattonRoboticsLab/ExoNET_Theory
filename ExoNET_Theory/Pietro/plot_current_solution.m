function plot_current_solution(params, cost, improvement_num)
% Real-time plotting of current best solution
persistent fig_handle;

if isempty(fig_handle) || ~isvalid(fig_handle)
    fig_handle = figure('Name', 'Real-time Optimization Progress', 'Position', [100, 100, 800, 600]);
end

figure(fig_handle);
subplot(2, 2, 1);
bar(params(1:min(10, length(params))));
title(sprintf('Best Parameters (Improvement #%d)', improvement_num));
xlabel('Parameter Index');
ylabel('Value');
grid on;

subplot(2, 2, 2);
plot(improvement_num, cost, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
if improvement_num > 1
    hold on;
end
title('Cost Evolution');
xlabel('Improvement Number');
ylabel('Cost');
grid on;

drawnow;
end

