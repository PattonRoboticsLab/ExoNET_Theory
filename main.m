% main: main script to do exoNet
% patton's main program. 
% this entire directory was modified from gravityProcess_mixedDevice code

clear; close all; clc; fprintf('\n ~ MAIN script:  ~ \n')  % begin

setUpArm % sets basic params

task=menu('Choose:','anti-gravity','EA','Draw your own'); % exoDesign

switch task
  
  case 1 % gravity comp
  setUpGrav                                        % set variables & plots 
  [p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,options.nTries); % <-- globalOptim
  save gravity;  playwav('SHOOP.WAV');

  case 2  %% EA
  setUpEA                                       % set variables & plots
  [p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,options.nTries); % <-- globalOptim
  save EAField; playwav('SHOOP.WAV');

  case 3 %% Draw your own
  setUpDrawn
  [p,c,TAUs]=robustOpto(p0,PHIs,Bod,Pos,options.nTries); % <-- globalOptim
  evalThesePoints  % also plot vectors for extra points not in optimization
  save drawnField; playwav('SHOOP.WAV');

  otherwise
    disp('not developed yet.   ')

end % END switch

fprintf(' end MAIN script. \n')

