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
%   if (.0099 < R) && (R < .01001)
%       cost = cost./1000;
%   elseif (.0195<R) && (R<.02005)
%       cost = cost./1000;
%   elseif (.0295<R) && (R<.03005)
%       cost = cost./1000;
%   elseif (.0395 < R) && (R < .04005)
%       cost = cost./1000;
%   elseif (.0495 < R) && ( R<.05005)
%       cost = cost./1000;
%   elseif (.0595<R) && (R<.06005)
%       cost = cost./1000;
%   elseif (.0695 < R) && (R<.07005)
%       cost = cost./1000;
%   elseif (.0795 < R) && (R<.08005)
%       cost = cost./1000;
%   elseif (.0895 < R) && (R<.09005)
%       cost = cost./1000;     
%   elseif R < 0
%       cost = cost.*1000;
%   end
  isLarger=abs(R)>R_max;
  LargerBy=(abs(R)-R_max)*isLarger;
  cost=cost+lamda*LargerBy^3;  % keep it short
  
end

% thet_max = 6.28319;
% for i = 2:3:length(p)
%    theta = p(i);
%    if (.52 < theta) && (theta < .53)
%        cost = cost./100;
%    elseif (1.04 < theta) && (theta <1.05)
%        cost = cost./100;
%    elseif (1.565 < theta) && (theta <1.58)
%        cost = cost./100;
%    elseif (2.09 < theta) && (theta < 2.1)
%        cost = cost./100;
%    elseif (2.61 < theta) && (theta < 2.62)
%        cost = cost./100;
%    elseif (3.14 < theta) && (theta < 3.16)
%        cost = cost./100;
%    elseif (3.66 < theta) && (theta<3.67)
%        cost = cost./100;
%    elseif (4.18 < theta) && (theta < 4.19)
%        cost = cost./100;
%    elseif (4.65 < theta) && (theta < 4.72)
%        cost = cost./100;
%    elseif (5.23 < theta) && (theta < 5.24)
%        cost = cost./100;
%    elseif (5.75 < theta) && (theta < 5.76)
%        cost = cost./100;
%    elseif theta < 0
%        cost = cost.*100;
%    elseif theta > thet_max
%        cost = cost.*100;
%    end
  
end
    
