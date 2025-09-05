function print_solution_parameters(params, indexswivel, title)
% Formatted parameter display
fprintf('\n%s:\n', title);
fprintf('=====================================\n');
fprintf('%-12s %8s %8s %8s %8s %8s %8s %8s %8s %8s\n', ...
    'Element', 'R', 'Theta', 'Y', 'L0', 'Perc', 'K', 'K_rec', 'L0_rec', 'L_pre_rec');
fprintf('-------------------------------------\n');
fprintf('%-12s ', 'Row 1:');
fprintf('%8.3f ', params(1:indexswivel)');
fprintf('\n%-12s ', 'Row 2:');
fprintf('%8.3f ', params(indexswivel+1:indexswivel*2)');
fprintf('\n%-12s %8s %8s %8s\n', 'Wrist:', 'L0_wr', 'Perc_wr', 'K_wr');
fprintf('%-12s ', '');
fprintf('%8.3f ', params(indexswivel*2+1:end)');
fprintf('\n=====================================\n');
end