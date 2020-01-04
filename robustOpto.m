% robustOpto:  Make a roboust effort to find the global optimization
% bestP=robustOpto(p0,PHIs,Bod,Pos,nTries)
% returns the best choice of several random initial guesses
% ~~ BEGIN: ~~

function [bestP,bestCost,TAUs]=robustOpto(p0,PHIs,Bod,Pos,nTries)

%% setup
fprintf('Optimization: '); drawnow; pause(.1);    % update display
global TAUsDesired ProjectName PHIsWorkspace PosWorkspace
bestP=randn(1,length(p0));                        % PICK RANDOM init
bestCost=1e5;                                     % init high
L0LoHi=[.1 5]; RLoHi=[.1 .01];                    % ranges

% plot initial guess
TAUs=exoNetTorques(bestP,PHIs);                   % calc improvedSolution
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');      % PLOT desired field
subplot(1,2,1); ax1=axis();
subplot(1,2,2); ax2=axis();            
plotVectField(PHIs,Bod,Pos,TAUs,.95*[1 1 1]);             % initial guess 
subplot(1,2,1); axis(ax1); subplot(1,2,2); axis(ax2);
title(ProjectName); drawnow; pause(.1);           % & show a tileupdates display

%% loop multiple tries
for i=1:nTries
  fprintf('Opt#%d..',i);
  p0=randn(1,length(p0))+bestP;                   % PICK RANDOM surround
  [p,c]=fminsearch('cost',p0);                    % OPTIMIZATION !
  if c<bestCost,                                  % if lower cost
    fprintf(' c=%g, ',c); fprintf(' p=%g ',p);    % display
    bestCost=c; bestP=p;                          % update best 
    TAUs=exoNetTorques(p,PHIs);                   % calc improvedSolution
    plotVectField(PHIs,Bod,Pos,TAUs,[.8 .9 .9]);  % improvedSolution Grey
    subplot(1,2,1); axis(ax1);                    %
    subplot(1,2,2); axis(ax2);                    %
    title(ProjectName);                           % & show a tile
    fprintf('\n'); drawnow; pause(.1);            % updates display
  else fprintf(' (not an improvement) \n ');
  end 
end

%% WRAP UP OPTO with one last run
[p,c]=fminsearch('cost',bestP);                       % last OPTIM @ best
if c<bestCost,
  bestCost=c; bestP=p;                                % updateW/better cost
  fprintf(' c=%g, ',c); fprintf(' p=%g ',p);          % display
end 

%% PLOTS
subplot(1,2,1); drawBody2(Bod.pose,Bod);              % cartoon man, posed
drawExonets(bestP,Bod.pose);                          % exonet lineSegs
TAUs=exoNetTorques(bestP,PHIs);                       % solution calc
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');          % desired again
plotVectField(PHIs,Bod,Pos,TAUs,'b');                 % plot solution 
% PHIs=PHIsWorkspace; Pos=PosWorkspace;               % add fullWorkspace
% TAUs=exoNetTorques(p,PHIs);                         % field @these 
% plotVectField(PHIs,Bod,Pos,TAUs,'b');               % also plot these
subplot(1,2,2); axis(ax2); subplot(1,2,1); axis(ax1); % zoom
title(ProjectName); drawnow; pause(.1);               % show tile & update

eval(['save ' ProjectName]);
eval(['print -depsc2 ' ProjectName]);

