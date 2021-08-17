% evaluate cost function for desired torques TAUs at positions PHIs
% ~~ BEGIN PROGRAM: ~~

function cost=cost(p);

global PHIs TAUsDesired



%     p(1) = .05;
%     p(4) = .05;
%     p(7) = .05;
%     p(10) = .05;
%     p(13) = .05;
%     p(16) = .05;
%     p(19) = .05;
%     p(22) = .05;
%     p(25) = .05;
    

e=TAUsDesired-exoNetTorques_n(p,PHIs); % torques errors

cost= mean(sum(e.^2))  % Sum of squares of all errors at all positons

%% REGULARIZARION: soft contraint: L0 if less than realistic amount %
loL0Limit= .05; % realistic amount 
for i=3:3:length(p) % L0 is every third
 L0=p(i);
 ifShorter=L0<loL0Limit;
 shorterBy=(loL0Limit-L0)*ifShorter;
 cost=cost+1e3*shorterBy;
end

% for i = 1:3:length(p)
%     if p(i) < 0.0499
%         cost = cost + 1e5;
%     end
%     
%     if p(i) > 0.0501
%         cost = cost + 1e5;
%     end
% end

%% REGULARIZARION: soft contraint: r less than realistic amount %
% loRLimit= 0.01; % realistic amount 
% for i=1:3:length(p) % L0 is every third
%  R=p(i);
%  ifShorter=R<loRLimit;
%  shorterBy=(loRLimit-R)*ifShorter;
%  cost=cost+1e3*shorterBy;
% end
