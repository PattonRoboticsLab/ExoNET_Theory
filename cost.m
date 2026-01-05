% cost: evaluate cost function for ExoNET at positions PHIs
%% ~~ BEGIN PROGRAM: ~~
function c=cost(p)

global PHIs TAUsDesired Exo
lamda=1e6;
dontPlotIt=0;
e=TAUsDesired-exoNetTorques(p,PHIs,dontPlotIt); % torques errors
c=sum(sum(e.^2));  % Sum of squares of all errors at all positons

%% REGULARIZARION: soft contraint: all L0 if less than realistic amount %
loL0Limit= .05; % realistic amount 
for i=3:3:length(p) % L0 is every third
 L0=p(i);
 ifShorter=L0<loL0Limit;
 shorterBy=(loL0Limit-L0)*ifShorter;
 c=c+lamda*shorterBy;
end

%% Enforce soft constraints on the parameters (if preSet in Setup)
if ~exist('pConstraint','var') % default
    for i = 1:length(p) % loop thru each parameter constraint
        isLow = p(i) < Exo.pConstraint(i,1);
        lowBy = (Exo.pConstraint(i,1)-p(i))*isLow; % how low
        isHi = p(i) > Exo.pConstraint(i,2);
        hiBy = (p(i)-Exo.pConstraint(i,2))*isHi; % how high
        c = c + lamda*lowBy^3; % punishment - you can change value of exponent
        c = c + lamda*hiBy^3;  % punishment
    end    
end
