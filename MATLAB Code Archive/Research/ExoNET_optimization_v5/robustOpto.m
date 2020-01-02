% robustOpto make a roboust effort to find the global optimization
% bestP=robustOpto(p0,PHIs,Bod,Pos,nTries)
% returns the best choice of several random initial guesses
% ~~ BEGIN PROGRAM: ~~


function [bestP,bestCost,TAUs]=robustOpto(p0,PHIs,Bod,Pos,nTries)

fprintf('Optimization: '); drawnow; pause(.1);  % update display
bestCost=1e16; % init high

for i=1:nTries
  fprintf('Opt#%d..',i)
  theta_high = pi;
  theta_low = -pi;
  r_length_high = 10.00;
  r_length_low = .1;
  rad_high = .10;
  rad_low = .01;
  
     for i = 1:3
         rad(i) = (rad_high-rad_low).*randn(1,1);
         theta(i) = randn(1,1);
         r_length(i) = (r_length_high - r_length_low).*randn(1,1);
         p0(i*2-1+0) = rad(i);
         p0(i*2-1+1) = theta(i);
         p0(i*2-1+2) = r_length(i);
     end
  %p0=randn(1,length(p0));                       % PICK RANDOM init
  [p,c]=fminsearch('cost',p0);                  % OPTIMIZATION !
  if c<bestCost, 
    bestCost=c; bestP=p;
    fprintf(' UPDATED: cost=%g, ',c); p
  end % store if good
  TAUs=exoNetTorques(p,PHIs);                   % solution calc  
  plotVectField(PHIs,Bod,Pos,TAUs,'y');         % plot solution
  drawnow; pause(.1);                           % update display
end
TAUs=exoNetTorques(bestP,PHIs);                     % solution calc
plotVectField(PHIs,Bod,Pos,TAUs,'b');           % plot solution 
