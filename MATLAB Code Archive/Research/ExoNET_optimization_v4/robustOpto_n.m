% robustOpto make a robust effort to find the global optimiation
% bestP = robustOpto(p0, PHIs, Bod, Pos, nTries)
% returns the best choice of several random initial guesses

%% begin program

function [bestP, bestCost, TAUs] = robustOpto_n(p0, PHIs, Bod, Pos, nTries)

fprintf('Optimization: '); drawnow; pause(.1); %update display
bestCost = 1e16; %initial guess is high

for i = 1:nTries
    fprintf('Opt#%d...',i)          % PICK RANDOM init
     %p0 = randn(1,length(p0));
a = pi;
b = -pi;
c = 10.00;
d = .1;
     for i = 1:3
         z(i) = randn(1,1);
         y(i) = (c-d).*randn(1,1);
         p0(i*2-1) = z(i);
         p0(i*2) = y(i);
     end
     
         


%     p0(1) = .05;
%     p0(4) = .05;
%     p0(7) = .05;
%     p0(10) = .05;
%     p0(13) = .05;
%     p0(16) = .05;
%     p0(19) = .05;
%     p0(22) = .05;
%     p0(25) = .05;
    [p,c] = fminsearch('cost',p0);  % OPTIMIZATION !
    if c < bestCost,
        bestCost = c; bestP = p;
        fprintf(' UPDATED: cost = %g, ', c);
    end % store if good
    
    TAUs = exoNetTorques_n(p,PHIs);           % solution calc
    plotVectField_n(PHIs,Bod,Pos,TAUs,'y'); % plot solution
    drawnow; pause(.1);                     % update display
end

TAUs = exoNetTorques_n(bestP,PHIs);
plotVectField_n(PHIs,Bod,Pos,TAUs,'b');     % plot solution

    