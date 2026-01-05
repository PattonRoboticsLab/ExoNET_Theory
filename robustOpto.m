% robustOpto: manage a global optimization for
% bestP=robustOpto(p0,PHIs,Bod,Pos,nTries)
% several random initial guesses, plots results, & returns best choice 
% ~~ BEGIN PROGRAM: ~~


function [bestP,bestCost,TAUs]=robustOpto(p0,PHIs,Bod,Pos,nTries)

%% init
global TAUsDesired
bestCost=1e16; % init high
p0=0*p0;
plotIt='plotIt';
fprintf('Beginning Optimization: \n')
pose=pi/180*[-60 70];   % one single sample pose for drawings


%% loop 4 robust
for i=1:nTries
  fprintf('Opt#%d..',i)
  p0=randn(1,length(p0));                       % PICK RANDOM init
  [p,c]=fminsearch('cost',p0);                  % OPTIMIZATION !
  if c<bestCost, 
    fprintf('new c=%g, ',c); bestCost=c; bestP=p% update with better cost 
    TAUs=exoNetTorques(p,PHIs);                 % solution calc
    %plotVectField(PHIs,Bod,Pos,TAUs,'y');       % plot solution
  end 
  drawnow; pause(.1);                           % update display
end
[p,c]=fminsearch('cost',bestP);                 % last OPTIMIZATION @ best
if c<bestCost,
  fprintf('bettr c=%g, ',c); bestCost=c; bestP=p% update with better cost
  TAUs=exoNetTorques(p,PHIs,plotIt);            % solution calc
end 
fprintf('\n Optimization complete \n');

%% results plotting
subplot(1,2,1); 
drawBody3(pose, Bod);    % draw at one posture
drawExonets(bestP,pose);            % draw exonets as lineSegs
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');    % desired- see it again 
plotVectField(PHIs,Bod,Pos,TAUs,'b');           % plot solution 

