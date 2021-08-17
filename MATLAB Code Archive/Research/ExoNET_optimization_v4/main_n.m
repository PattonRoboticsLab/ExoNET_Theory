%% Main.m: main script file to create figures and run optimization

%% Begin
clear all;
close all;
clc;

fprintf('\n ~ MAIN script: ~ \n')

setup_n; %call all initial parameters and variables

%% gravity comp
figure(1);
fprintf('\n ~ Weight-comp for 2-joint multimarionet \n')
TAUsDesired = weightEffect_n(Bod,Pos);                     %set torques2cancelGrav
plotVectField_n(PHIs,Bod,Pos,TAUsDesired,'r');             % plot weight field
[p,c,TAUs] = robustOpto_n(p0,PHIs,Bod,Pos,options.nTries); % <-- ! global optimization
plotVectField_n(PHIs,Bod,Pos,TAUsDesired,'r');             % weight again to see it
plotVectField_n(PHIs,Bod,Pos,TAUs,'b');                    % plot solution
% 
drawnow; pause(.1); %updates display

%%
fprintf(' end MAIN script. \n')

%% EA
% fprintf('\n ~  EA  for 2-joint multimarionet  \n')
% figure(2);
% [TAUsDesired,PHIs,Pos]=eaField(Bod);                % set torques2cancelGrav
% plotVectField_n(PHIs,Bod,Pos,TAUsDesired,'r');        % plot weight field 
% [p,c,TAUs]=robustOpto_n(p0,PHIs,Bod,Pos,options.nTries); % <-- ! global optim 
% plotVectField_n(PHIs,Bod,Pos,TAUsDesired,'r');        % weight again to see it
% plotVectField_n(PHIs,Bod,Pos,TAUs,'b');               % plot solution 
% 
% drawnow; pause(.1);