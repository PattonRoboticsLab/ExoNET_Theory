% main: main script to do exoNet
% patton's main program. use the other main to do carella's
% this entire directory was modified from gravityProcess_mixedDevice code

%% begin
clear; close all; clc; 
fprintf('\n ~ MAIN script:  ~ \n')
setUp % set most variables and plots; 

%gravity comp
% fprintf('\n _____  Weight-comp : ____ \n')
% figure(1);
% TAUsDesired=weightEffect(Bod,Pos)                 % set torques2cancelGrav
% plotVectField_1(PHIs,Bod,Pos,TAUsDesired,'r');        % plot desired field 
% [p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,options.nTries); % <-- ! global optim
% figure(1);                                        
% plotVectField_1(PHIs,Bod,Pos,TAUsDesired,'r');        % weight- see it again 
% plotVectField(PHIs,Bod,Pos,TAUs,'b');          
% %      plot solution
% subplot(1,2,1); drawBody2(phiPose, Bod);            % draw again 
% drawExonets(p,phiPose);                             % exonets as lineSegs
% suptitle('Gravity Compensating Field');               
% drawnow; pause(.1);                                 % updates display
% % save gravity

%% EA
% fprintf('\n _____  EA field :  _____ \n')
%  figure(2);
% PHIsWorkspace=PHIs;  
% [TAUsDesired,PHIs,Pos]=eaField(Bod);                % set torques2cancelGrav
% plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');        % plot desired field 
% [p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,options.nTries); % <-- ! global optim 
% figure(2);    subplot(1,2,1);                                    
% plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');        % weight again to see it
% plotVectField(PHIs,Bod,Pos,TAUs,'b');               % plot solution
% PHIs=PHIsWorkspace;                                 % PHIS of fullWorkspace
% Pos=forwardKin(PHIs,Bod);                           % positions assoc w/ 
% TAUs=exoNetTorques(p,PHIs);                         % torques @these points
% plotVectField(PHIs,Bod,Pos,TAUs,'b');               % plot these  
% subplot(1,2,1); drawBody2(phiPose, Bod);            % draw body again at one posture
% drawExonets(p,phiPose)                              % exonets as lineSegs
% suptitle('Error Augmenting Field'); 
% drawnow; pause(.1);   % updates display
% save EAField

%Single Attractor 
% fprintf('\n _____  Single Attractor field :  _____ \n')
% figure(3)
% PHIsWorkspace=PHIs;
% [TAUs_1,PHIs,Pos]=SingleAttractor(Bod);           % set torques2cancelGravy
% plotAttractorField(PHIs,Bod,Pos,TAUs_1,'r');        % plot desired field 
% [p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,options.nTries); % <-- ! global optim 
% figure(3);    subplot(1,2,1);                                  
% %plotAttractorField(PHIs,Bod,Pos,TAUs,'r');        % weight again to see it
% plotVectField(PHIs,Bod,Pos,TAUs,'b');               % plot solution
% %PHIs=PHIsWorkspace;                                 % PHIS of fullWorkspace
% Pos=forwardKin(PHIs,Bod);                           % positions assoc w/ 
% TAUs=exoNetTorques(p,PHIs);                         % torques @these points
% plotVectField(PHIs,Bod,Pos,TAUs,'b');               % plot these  
%  subplot(1,2,1); drawBody2(phiPose, Bod);            % draw body again at one posture
%  drawExonets(p,phiPose)                              % exonets as lineSegs
% % suptitle('Single Attractor'); 
% % drawnow; pause(.1);   % updates display
% %save Single Attractor

% %% Dual Attractor 
% fprintf('\n _____  Dual Attractor field :  _____ \n')
% figure(4);
% PHIsWorkspace=PHIs;
% [TAUs_1,PHIs,Pos]=DualAttractor(Bod);           % set torques2cancelGrav
% plotAttractorField(PHIs,Bod,Pos,TAUs_1,'r');        % plot desired field 
% [p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,options.nTries); % <-- ! global optim 
%  figure(4);    subplot(1,2,1);                                  
% % plotAttractorField(PHIs,Bod,Pos,TAUs,'r');        % weight again to see it
%   plotVectField(PHIs,Bod,Pos,TAUs,'b');               % plot solution
% % PHIs=PHIsWorkspace;                                 % PHIS of fullWorkspace
%  Pos=forwardKin(PHIs,Bod);                           % positions assoc w/ 
%  TAUs=exoNetTorques(p,PHIs);                         % torques @these points
%  plotVectField(PHIs,Bod,Pos,TAUs,'b');               % plot these  
%  subplot(1,2,1); drawBody2(phiPose, Bod);            % draw body again at one posture
%  drawExonets(p,phiPose)                              % exonets as lineSegs
% suptitle('Dual Attractor'); 
% drawnow; pause(.1);   % updates display
%save Dual Attractor
%%

%% Limit Push Field
fprintf('\n _____  Limit Push field :  _____ \n')
figure(5);
PHIsWorkspace=PHIs;
[TAUs_1,PHIs,Pos]=LimitPush(Bod);           % set torques2cancelGrav
plotAttractorField(PHIs,Bod,Pos,TAUs_1,'r');        % plot desired field 
[p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,options.nTries); % <-- ! global optim 
 figure(5);    subplot(1,2,1);                                  
plotAttractorField(PHIs,Bod,Pos,TAUs,'r');        % weight again to see it
  plotVectField(PHIs,Bod,Pos,TAUs,'b');               % plot solution
PHIs=PHIsWorkspace;                                 % PHIS of fullWorkspace
 Pos=forwardKin(PHIs,Bod);                           % positions assoc w/ 
 TAUs=exoNetTorques(p,PHIs);                         % torques @these points
 %plotVectField(PHIs,Bod,Pos,TAUs,'b');               % plot these  
 subplot(1,2,1); drawBody2(phiPose, Bod);            % draw body again at one posture
 drawExonets(p,phiPose)                              % exonets as lineSegs
suptitle('Limit Push'); 
drawnow; pause(.1);   % updates display
%save Dual Attractor
fprintf(' end MAIN script. \n')
