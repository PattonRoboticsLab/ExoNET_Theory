% main: main script to do exoNet
% patton's main program. use the other main to do carella's
% this entire directory was modified from gravityProcess_mixedDevice code
 
%% begin
clear; close all; clc; 
fprintf('\n ~ MAIN script:  ~ \n')
setUp % set most variables and plots; 
 
%% gravity comp
fprintf('\n _____  Weight-comp : ____ \n')
figure(1);
TAUsDesired=weightEffect(Bod,Pos);                % set torques2cancelGrav
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');      % plot weight field 
[p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,options.nTries); % <-- ! global optim 
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');      % weight again to see it
plotVectField(PHIs,Bod,Pos,TAUs,'b');             % plot solution
subplot(1,2,1); drawExonets(p,phiPose);
% suptitle('Error Augmenting Field'); 
drawnow; pause(.1);                                 % updates display
save gravity
 
%% EA
fprintf('\n _____  EA field :  _____ \n')
figure(2);
PHIsWorkspace=PHIs;  
[TAUsDesired,PHIs,Pos]=eaField(Bod);                % set torques2cancelGrav
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');        % plot weight field 
[p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,options.nTries); % <-- ! global optim 
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');        % weight again to see it
plotVectField(PHIs,Bod,Pos,TAUs,'b');               % plot solution
 
PHIs=PHIsWorkspace;         % go back to a span of the full workspace
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations
TAUs=exoNetTorques(p,PHIs);                   % torques at these points
plotVectField(PHIs,Bod,Pos,TAUs,'b');               % plot solution 
subplot(1,2,1); drawExonets(p,phiPose)
% suptitle('Error Augmenting Field'); 
drawnow; pause(.1);   % updates display
save EA
 
%%
fprintf(' end MAIN script. \n')