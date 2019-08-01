% evaluate cost function for desired torques TAUs at positions PHIs
% ~~ BEGIN PROGRAM: ~~
% Small Edit to See if Git Works
function cost=cost(p)

global PHIs TAUsDesired TAUs_1
lamda=1e7;

%TAUs_1, exoNetTorques(p,PHIs), p,PHIs; 

%For Gravity Compensation and EA Field
%e=TAUsDesired-exoNetTorques(p,PHIs); % torques errors

%For All Other Fields
e = TAUs_1 - exoNetTorques(p,PHIs);
cost=mean(sum(e.^2));  % Sum of squares of all errors at all positons

%% REGULARIZARION: soft contraint: all L0 if less than realistic amount %
loL0Limit= .05; % realistic amount 
for i=3:3:length(p) % L0 is every third
 L0=p(i);
 ifShorter=L0<loL0Limit;
 shorterBy=(loL0Limit-L0)*ifShorter;
 cost=cost+lamda*shorterBy;
end

%% REGULARIZARION: soft contraint: all r less than realistic amount %
R_max= .1;                                  % practical max 
for i=1:3:length(p) % L0 is every third
  R=p(i);
  isLarger=abs(R)>R_max;
  LargerBy=(abs(R)-R_max)*isLarger;
  cost=cost+lamda*LargerBy^3;  % keep it short
end