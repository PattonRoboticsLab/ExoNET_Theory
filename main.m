% main: main script to do exoNet
% patton's main program. use the other main to do carella's
% this entire directory was modified from gravityProcess_mixedDevice code

%% begin
clear; close all; clc; 
fprintf('\n ~ MAIN script:  ~ \n')
disp('choose from menu...')
fieldType=menu('Choose a field to approximate:' ...
               , 'WeightCompensation' ...
               , 'ErrorAugmentation' ...
               , 'SingleAttractor' ...
               , 'DualAttractor' ...
               , 'LimitPush' ...
               , 'EXIT');        
setUp % set most variables and plots; 

switch fieldType
  case 1 % gravity Compensation
    [TAUsDesired,PHIs,Pos]=weightEffect(Bod,Pos);       % cancelGravity
    [p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,nTries);      % ! global optim

  case 2 % EA
    [TAUsDesired,PHIs,Pos]=eaField(Bod);                % set EA field
    [p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,nTries);      % ! global optim

  case 3  % SingleAttractor
    [TAUsDesired,PHIs,Pos]=SingleAttractor(Bod);        % torques
    [p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,nTries);      % ! global optim

  case 4  % DualAttractor
    [TAUsDesired,PHIs,Pos]=DualAttractor(Bod);               % torques
    [p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,nTries);      % ! global optim
    
  case 5
    [TAUsDesired,PHIs,Pos]=LimitPush(Bod);                   % torques
    [p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,nTries);      % ! global optim

  otherwise
    disp('exiting..'); close all
    
end % END switch


fprintf(' end MAIN script. \n')
