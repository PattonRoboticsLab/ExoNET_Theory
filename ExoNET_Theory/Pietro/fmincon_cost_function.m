function costBO = cost_BO(x, TAUsDesired, Exo, Pos, options, lb, ub, matrix_size)
    % Cost function for BO
    p = zeros(numel(lb), 1);
    for i = 1:numel(lb)
        p(i) = x.(['x' num2str(i)]);
    end
    p0_3D = reshape(p, matrix_size);    
    [~, costBO] = fmincon(@(p) cost(p, TAUsDesired, Exo, Pos), p0_3D, [], [], [], [], lb, ub, [], options);
end