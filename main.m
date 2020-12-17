% main: main script to do exoNet
% patton's main program. use the other main to do carella's

%% begin
clear all; close all; clc; 
fprintf('\n ~ MAIN script:  ~ \n')  
setUp % set most variables and plots in a SCRIPT 

switch fieldType
  case 1 % gravity Compensation
    [TAUsDesired,PHIs,Pos]=weightEffect(Bod,Pos);       % determine desired
    [p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim
    %show_tensionstretch(PHIs,Bod,p)
    

  case 2 % EA
    [TAUsDesired,PHIs,Pos]=eaField(Bod);                % determine desired
    [p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim
    show_tensionstretch(PHIs,Bod,p)

  case 3  % SingleAttractor
    [TAUsDesired,PHIs,Pos]=SingleAttractor(Bod);        % determine desired
    [p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim
    show_tensionstretch(PHIs,Bod,p)

  case 4  % DualAttractor
    [TAUsDesired,PHIs,Pos]=DualAttractor(Bod);          % determine desired
    [p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim
    show_tensionstretch(PHIs,Bod,p)
    
  case 5
    [TAUsDesired,PHIs,Pos]=LimitPush(Bod);              % determine desired
    [p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim
    show_tensionstretch(PHIs,Bod,p)
% - Danny says hi
  case 6
    setUpLeg
    [p,c,TAUs,costs]=robustOptoLeg(PHIs,BODY,Pos,Exo,nTries)     % ! global optim
    show_tensionstretch(PHIs,Bod,p)

    
  otherwise
    disp('exiting..'); close all
    
end % END switch


fprintf(' end MAIN script. \n')
