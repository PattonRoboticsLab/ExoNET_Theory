% ***********************************************************************
% Evaluate the cost function for the torques TAUs at positions PHIs
% ***********************************************************************

function [c,meanErr] = costLeg(p)

%% Setup
global PHIs TAUsDESIRED EXONET

e = TAUsDESIRED - exoNetTorquesLeg(p,PHIs); % torques errors at each operating point
c = mean(sum(e.^2));                        % to sum the squares of the errors at all positions
meanErr = norm(mean(e));                    % average error


%% Penalize R
for i = 1:3:length(p) % loop thru each R parameter
    c = c + p(i);     % penalize R to drive it to zero
    isLow = p(i) < EXONET.pConstraint(i,1);       % if is lower than the min
    lowBy = (EXONET.pConstraint(i,1)-p(i))*isLow; % how low
    isHi = p(i) > EXONET.pConstraint(i,2);        % if is higher than the max
    hiBy = (p(i)-EXONET.pConstraint(i,2))*isHi;   % how high
    c = c + 10*lowBy; % increase cost if is lower than the min
    c = c + 100*hiBy; % increase cost if is higher than the max
end


%% Penalize L0
for i = 3:3:length(p) % loop thru each L0 parameter
    isNeg = p(i) < 0; % if is negative
    hiNeg = 1000*isNeg;
    c = c + hiNeg;    % increase cost if is negative
    isLow = p(i) < EXONET.pConstraint(i,1);       % if is lower than the min
    lowBy = (EXONET.pConstraint(i,1)-p(i))*isLow; % how low
    isHi = p(i) > EXONET.pConstraint(i,2);        % if is higher than the max
    hiBy = (p(i)-EXONET.pConstraint(i,2))*isHi;   % how high
    c = c + 10*lowBy; % increase cost if is lower than the min
    c = c + 100*hiBy; % increase cost if is higher than the max
end

end