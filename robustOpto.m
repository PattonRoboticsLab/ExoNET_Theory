% robustOpto make a roboust effort to find the global optimization
% bestP=robustOpto(p0,PHIs,Bod,Pos,nTries)
% returns the best choice of several random initial guesses
% ~~ BEGIN PROGRAM: ~~
%% Line 24 Must be changed (22 changed to 13 or 1 based on number of elements)


function [bestP,bestCost,TAUs]=robustOpto(p0,PHIs,Bod,Pos,nTries)

fprintf('Optimization: '); drawnow; pause(.1);  % update display
bestCost=1e5; % init high

%p0=0*p0;

for i=1:nTries
  fprintf('Opt#%d..',i)
  
  % resting length range
  r_length_high = 5.1;
  r_length_low = .5;
  % radius range
  rad_high = .05;
  rad_low = .0100;
  
  %13 for 3 elements, 22 for 5 elements, 3 for 1 element
     for i = 1:3
         rad(i) = abs((rad_high-rad_low).*randn(1,1)+rad_low);
         theta(i) = randn(1,1);
         r_length(i) = abs((r_length_high - r_length_low).*randn(1,1)+r_length_low);
         p0(i*2-1+0) = rad(i);
         p0(i*2-1+1) = theta(i);
         p0(i*2-1+2) = r_length(i);
     end
     %p0
   p0=randn(1,length(p0));                       % PICK RANDOM init
  [p,c]=fminsearch('cost',p0);                  % OPTIMIZATION !
  if c<bestCost, 
    fprintf('new c=%g, ',c); bestCost=c; bestP=p% update with better cost 
  
    end 
  TAUs=exoNetTorques(p,PHIs);                   % solution calc  
  %plotVectField(PHIs,Bod,Pos,TAUs,'y');         % plot solution
  %drawnow; pause(.1);                           % update display
end
[p,c]=fminsearch('cost',bestP);                 % last OPTIMIZATION @ best
if c<bestCost,
  fprintf('bettr c=%g, ',c); bestCost=c; bestP=p% update with better cost
end 
TAUs=exoNetTorques(bestP,PHIs);                 % solution calc
plotVectField(PHIs,Bod,Pos,TAUs,'b');           % plot solution 
