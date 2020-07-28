% ***********************************************************************
% Evaluate the cost function for desired torques TAUs at positions PHIs
% ***********************************************************************

function [c,meanErr] = cost(p)

%% Setup
global PHIs TAUsDesired Exo

lambda = 100;
e = TAUsDesired - exoNetTorques(p,PHIs); % torques errors at each operating point
c = sum(sum(e.^2)); % to sum the squares of the errors at all positions
meanErr = norm(mean(e)); % average error


%% Enforce soft constraints on the parameters (if preSet in Setup)
if ~exist('pConstraint','var') % default
    for i = 1:length(p) % loop thru each parameter constraint
        isLow = p(i) < Exo.pConstraint(i,1);
        lowBy = (Exo.pConstraint(i,1)-p(i))*isLow; % how low
        isHi = p(i) > Exo.pConstraint(i,2);
        hiBy = (p(i)-Exo.pConstraint(i,2))*isHi; % how high
        c = c + lambda*lowBy^3; % quadratic punishment - you can change value of exponent
        c = c + lambda*hiBy^3;  % quadratic punishment
    end
end

%% penalize R to drive to zero
for i=1:3:length(p) % loop thru each R parameter 
    if (p(i) < 0) || (p(i) > .15)
        c=c+lambda^2;%p(i); 
    else
        c = c;
        
end

% %% REGULARIZARION: soft contraint: all L0 if less than realistic amount %
% loL0Limit= .05;           % realistic amount 
% for i=3:3:length(p)       % L0 is every third
%  L0=p(i);
%  ifShorter=L0<loL0Limit; shorterBy=(loL0Limit-L0)*ifShorter;
%  cost=cost+lamda*shorterBy;
% end
% 
% %% REGULARIZARION: soft contraint: all r less than realistic amount %
% R_max= .1;                % practical max 
% for i=1:3:length(p)       % R0 is fist and every third
%   R=p(i);
%   isLarger=abs(R)>R_max;
%   LargerBy=(abs(R)-R_max)*isLarger;
%   cost=cost+lamda*LargerBy^3;  % keep it short
% end

%% Penalize R to drive to zero
% for i = 1:3:length(p) % loop thru each R parameter
%     c = c + p(i);
% end

end