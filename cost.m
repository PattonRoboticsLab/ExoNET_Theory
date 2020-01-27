% ***********************************************************************
% Evaluate the cost function for desired torques TAUs at positions PHIs
% ***********************************************************************

function [c,meanErr] = cost(p)

%% Setup
global PHIs TAUsDESIRED EXONET

lambda = 10;
e = TAUsDESIRED - exoNetTorques(p,PHIs); % torques errors at each operating point
c = mean(sum(e.^2)); % to sum the squares of the errors at all positons
meanErr = norm(mean(e)); % vector of average errors (not really a vector)


%% Enforce soft constraints on the paramters (if preSet in Setup)
if ~exist('pConstraint','var') % default
    for i = 1:length(p) % loop thru each parameter constraint
        isLow = p(i) < EXONET.pConstraint(i,1);
        lowBy = (EXONET.pConstraint(i,1)-p(i))*isLow; % how low
        isHi = p(i) > EXONET.pConstraint(i,2);
        hiBy = (p(i)-EXONET.pConstraint(i,2))*isHi; % how high
        c = c + lambda*lowBy; % quadratic punishment
        c = c + lambda*hiBy; % quadratic punishment
    end
end

end