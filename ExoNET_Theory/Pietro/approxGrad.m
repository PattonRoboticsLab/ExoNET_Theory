% Function to accelerate fmincon, evaluate the gradient of the cost
% evaluating the cost of the function of the following step
function grad = approxGrad(p, costFun, cAtP)
    delta = 1e-6;
    n = numel(p);
    grad = zeros(n, 1);
    
    if nargin < 3
        cAtP = costFun(p);  % calcola solo se non fornito
    end

    % Parallel loop
    parfor i = 1:n
        p_perturbed = p;
        p_perturbed(i) = p_perturbed(i) + delta;
        f_perturbed = costFun(p_perturbed);
        grad(i) = (f_perturbed - cAtP) / delta;
    end
    
end

