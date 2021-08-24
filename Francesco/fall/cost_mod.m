% ***********************************************************************
% Evaluate the cost function for desired torques TAUs at positions PHIs
% ***********************************************************************

function [c,meanErr] = cost_mod(p)
%% Setup
global PHIs TAUsDesired Exo error_norm 
lambda = 1000;
e = TAUsDesired - exoNetTorques_mod(p,PHIs); % torques errors at each operating point
Exo.E = e;
error_norm = sqrt(sum(e.^2,2));
c = sum(sum(e.^2)); % to sum the squares of the errors at all positions
meanErr = norm(mean(e)); % average error
%% Enforce soft constraints on the parameters (if preSet in Setup)
if ~exist('pConstraint','var') % default
    for i = 1:length(p) % loop thru each parameter constraint
        isLow = p(i) < Exo.pConstraint(i,1);
        lowBy = (Exo.pConstraint(i,1)-p(i))*isLow; % how low
        isHi = p(i) > Exo.pConstraint(i,2);
        hiBy = (p(i)-Exo.pConstraint(i,2))*isHi; % how high
        c = c + lambda*lowBy^3; % punishment - you can change value of exponent
        c = c + lambda*hiBy^3;  % punishment
    end    
end


%% Regularization for Stretch Ratio
for test_point = 1:size(e,1)
    for joint = 1:3
        for element = 1:size(p,1)
         L0 = p(joint+(element-1)*Exo.nParams+2);       % Extract L0
         stretch_ratio = Exo.Tdist(test_point, joint, element)/L0;                  % Calculate Ratio
         isHi = stretch_ratio > Exo.stretch_ratio_limit;    %boolean, if not true, isHi = 0
         hiBy = (stretch_ratio-Exo.stretch_ratio_limit)*isHi; % how high
         c = c + lambda*hiBy^3;  %  punishment
        end
    end
end

end