% evaluate cost function for desired torques TAUs at positions PHIs
% ~~ BEGIN PROGRAM: ~~

function cost=cost(p)

global PHIs TAUsDesired

e=TAUsDesired-exoNetTorques(p,PHIs); % torques errors

cost=sum(sum(e.^2));  % Sum of squares of all errors at all positons

%% REGULARIZARION: soft contraint: L0 if less than realistic amount %
loL0Limit= .05; % realistic amount 
for i=3:3:length(p) % L0 is every third
 L0=p(i);
 ifShorter=L0<loL0Limit;
 shorterBy=(loL0Limit-L0)*ifShorter;
 cost=cost+1e3*shorterBy;
end

%% REGULARIZARION: soft contraint: r less than realistic amount %
loRLimit= .01; % realistic amount 
for i=1:3:length(p) % L0 is every third
 R=p(i);
 ifShorter=R<loRLimit;
 shorterBy=(loRLimit-R)*ifShorter;
 cost=cost+1e3*shorterBy;
end
