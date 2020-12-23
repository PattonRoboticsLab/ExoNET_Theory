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

  case 6
    setUpLeg
    [p,c,TAUs,costs] = robustOptoLeg(PHIs,BODY,Position,EXONET,nTries);  % optimization
    showGraphTorquesLeg(percentageGaitCycle,TAUsDESIRED,TAUs)
    
    pp = p;
    fprintf('\n\n\n\n The Optimal Parameters for each Element are~~\n')
    n = 1;
    for i = 1:3:length(pp)  % for loop to print the values of the optimal parameters
        if abs(pp(i+1))>360 % to adjust the angle theta if it's higher than 360 degrees
            while abs(pp(i+1))>360
                pp(i+1) = sign(pp(i+1))*(abs(pp(i+1))-360);
            end
        end
        if pp(i)<0          % if r is negative
            pp(i) = pp(i)*(-1);
            pp(i+1) = pp(i+1)+180;
        end
        fprintf('\n Element %d\n',n)
        fprintf('\n r = %4.2f cm   theta = %4.2f deg   L0 = %4.2f cm\n',pp(i)*100,pp(i+1),pp(i+2)*100)
        n = n+1;
    end
    
    
    
  otherwise
    disp('exiting..'); close all
    
end % END switch


fprintf(' end MAIN script. \n')
