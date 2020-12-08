% *********************************************************************** 
% OPTIMIZATION: 
% Make a robust effort to find the global optimization 
% and return the best choice of several random initial guesses 
% *********************************************************************** 
 
function [bestP,bestCost,TAUs] = robustOptoLeg(PHIs,BODY,Position,EXONET,nTries) 
 
%% Setup 
fprintf('\n\n\n\n robustOpto~~\n') 
drawnow    % to update the figures in the graphic screen 
pause(0.1) % to pause for 0.1 seconds before continuing 
 
global TAUsDESIRED ProjectName 
 
ProjectName = 'Torque Approximation for Gait'; 
 
p0 = mean(EXONET.pConstraint');      % initial values of the parameters 
bestP = p0;                          % best parameters 
bestCost = 1e5;                      % best cost, initially high 
TAUs = exoNetTorquesLeg(bestP,PHIs); % initial guess for the solution 
 
 
%% Set the plot 
clf % to reset the figure 
 
subplot(1,2,1) 
title(ProjectName) 
drawBodyLeg(BODY); 
plotVectFieldLeg(PHIs,BODY,Position,TAUsDESIRED,'r'); % to plot the desired torque field in red 
 
subplot(1,2,1); ax1 = axis(); % to get axis zoom frame 
subplot(1,2,2); ax2 = axis(); % to get axis zoom frame 
 
plotVectFieldLeg(PHIs,BODY,Position,TAUs,0.9*[1 1 1]); % to plot the initial guess in grey 
plotVectFieldLeg(PHIs,BODY,Position,TAUsDESIRED,'r');  % to plot the desired torque field in red 
 
subplot(1,2,1) 
drawBodyLeg(BODY); 
drawExonetsLeg(bestP,BODY.pose); % to draw the ExoNET line segments 
 
subplot(1,2,1); axis(ax1); % to reframe the window 
subplot(1,2,2); axis(ax2); % to reframe the window 
 
title(ProjectName) 
drawnow; pause(0.1) % to show the plots 
 
 
%% Loop multiple optimization tries with simulated Annealing Perturbation 
fprintf('\n\n\n\n Begin Optimizations~~\n') 
tic 
for TRY = 1:nTries 
    fprintf('Opt#%d..',TRY); 
    [p,c] = fminsearch('costLeg',p0); % OPTIMIZATION 
    [p,c] = fminsearch('costLeg',p);  % OPTIMIZATION 
    if c < bestCost                   % if the cost is decreased 
        fprintf(' c=%g, ',c); p'      % to display the cost 
        bestCost = c;                 % to update the best cost 
        bestP = p;                    % to update the best parameters 
        TAUs = exoNetTorquesLeg(p,PHIs); % new guess for the torque field 
         
        % Update the plots 
        clf % to reset the figure 
        subplot(1,2,1) 
        drawBodyLeg(BODY); 
        drawExonetsLeg(bestP,BODY.pose); % to draw the ExoNET line segments 
        plotVectFieldLeg(PHIs,BODY,Position,TAUsDESIRED,'r');    % to plot the desired torque field in red 
        plotVectFieldLeg(PHIs,BODY,Position,TAUs,[0.8 0.9 0.9]); % to plot the improved solution in grey 
        fprintf('\n'); 
        drawnow; pause(0.1) % to update the display 
        title([ProjectName ', cost = ' num2str(c)]) 
        drawnow; pause(0.1) % to update the display 
    else 
        fprintf('\n\n (not an improvement)~~\n') 
    end 
    pKick = range(EXONET.pConstraint').*(nTries/TRY); % to simulate Annealing Perturbation 
    p0 = bestP + 1*randn(1,length(p0)).*pKick;        % to kick p away from its best value 
end 
toc 
 
 
%% Wrap up the Optimization with one last run starting at the best location 
fprintf('\n\n\n\n Final Optimization~~\n') 
[p,c] = fminsearch('costLeg',bestP); % last and best OPTIMIZATION 
if c < bestCost 
    bestCost = c; 
    bestP = p; 
    fprintf(' c=%g, ',c); p' 
else 
    fprintf('\n\n (not an improvement)~~\n') 
end 
[c,meanErr] = costLeg(bestP); meanErr 
 
 
%% Update the plots 
clf 
subplot(1,2,1) 
drawBodyLeg(BODY); 
drawExonetsLeg(bestP,BODY.pose);       % to draw the ExoNET line segments 
TAUs = exoNetTorquesLeg(bestP,PHIs); % to calculate the final solution 
plotVectFieldLeg(PHIs,BODY,Position,TAUsDESIRED,'r'); % to plot the desired torque field in red 
plotVectFieldLeg(PHIs,BODY,Position,TAUs,'b');        % to plot the best solution in blue 
 
subplot(1,2,1); axis(ax1); % to zoom the frame 
subplot(1,2,2); axis(ax2); % to zoom the frame 
title([ProjectName ', Average Error = ' num2str(meanErr)]); % to show the average error 
drawnow; pause(0.1) % to update the screen 
 
% % % % 
% put_figure(1, 0.02, 0.07, 0.95, 0.82); 
% for i = 1 : 106 
%     BODY.pose = [PHIs(i,1), PHIs(i,2)]; 
%     clf % to clear the previous plot 
%     drawBodyLeg(BODY.pose,BODY); 
%     drawExonetsLeg(bestP,BODY.pose); 
%     pause(0.0001) 
% end 
% % % % 
 
% eval(['save ' ProjectName]); 
% orient landscape 
% eval(['print -dpdf ' ProjectName]); 
 
end