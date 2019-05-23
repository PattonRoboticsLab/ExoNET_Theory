% robustOpto make a roboust effort to find the global optimization
% bestP=robustOpto(p0,PHIs,Bod,Pos,nTries)
% returns the best choice of several random initial guesses
% ~~ BEGIN PROGRAM: ~~


function [bestP,bestCost,TAUs]=robustOpto(p0,PHIs,Bod,Pos,nTries)

fprintf('Optimization: '); drawnow; pause(.1);  % update display
bestCost=1e16; % init high

for i=1:nTries
  fprintf('Opt#%d..',i)
  p0=randn(1,length(p0));                       % PICK RANDOM init
  
  [p,c]=fminsearch('cost',p0);                  % OPTIMIZATION !
  
  if c<bestCost, 
    bestCost=c; bestP=p;
    fprintf(' UPDATED: cost=%g, ',c); 
  end % store if good
  TAUs=exoNetTorques(p,PHIs);                   % solution calc  
  plotVectField(PHIs,Bod,Pos,TAUs,'y');         % plot solution
  drawnow; pause(.1);                           % update display
end
TAUs=exoNetTorques(bestP,PHIs);                     % solution calc
plotVectField(PHIs,Bod,Pos,TAUs,'b');           % plot solution 
