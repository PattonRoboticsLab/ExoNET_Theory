% main: main script to do exoNet
% patton's main program. use the other main to do carella's

%% begin
clear; close all; clc; 
fprintf('\n ~ MAIN script:  ~ \n')  
setUp % set most variables and plots in a SCRIPT 

switch fieldType
  case 1 % gravity Compensation
    [TAUsDesired,PHIs,Pos]=weightEffect(Bod,Pos);       % determine desired
    [p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim
    p

  case 2 % EA
    [TAUsDesired,PHIs,Pos]=eaField(Bod);                % determine desired
    [p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim

  case 3  % SingleAttractor
    [TAUsDesired,PHIs,Pos]=SingleAttractor(Bod);        % determine desired
    [p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim

  case 4  % DualAttractor
    [TAUsDesired,PHIs,Pos]=DualAttractor(Bod);          % determine desired
    [p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim
    
  case 5
    [TAUsDesired,PHIs,Pos]=LimitPush(Bod);              % determine desired
    [p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim

  case 6
    setUpLeg
    [p,c,TAUs]=robustOptoLeg(PHIs,BODY,Pos,Exo,nTries)     % ! global optim
    
  otherwise
    disp('exiting..'); close all
    
end % END switch


fprintf(' end MAIN script. \n')
