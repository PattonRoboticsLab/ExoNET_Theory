% main: main script to create a figure
% patton's main program. use the other main to do carella's
% this entire directory was modified from gravityProcess_mixedDevice code

%% begin
clear; close all; clc; 
fprintf('\n ~ MAIN script:  ~ \n')
setUp % set most variables and plots; 

%% gravity comp
figure(1);
fprintf('\n ~  Weight-comp  for 2-joint multimarionet  \n')
TAUsDesired=weightEffect(Bod,Pos);                % set torques2cancelGrav
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');      % plot weight field 
[p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,options.nTries); % <-- ! global optim 
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');      % weight again to see it
plotVectField(PHIs,Bod,Pos,TAUs,'b');             % plot solution

drawnow; pause(.1);   % updates display

%% EA
fprintf('\n ~  EA  for 2-joint multimarionet  \n')
figure(2);
[TAUsDesired,PHIs,Pos]=eaField(Bod);                % set torques2cancelGrav
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');        % plot weight field 
[p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,options.nTries); % <-- ! global optim 
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');        % weight again to see it
plotVectField(PHIs,Bod,Pos,TAUs,'b');               % plot solution 

drawnow; pause(.1);   % updates display


%%
fprintf(' end MAIN script. \n')
