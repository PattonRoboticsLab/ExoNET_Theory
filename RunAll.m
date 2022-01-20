%% begin
clear all; close all; clc; 
fprintf('\n ~ MAIN script:  ~ \n')  

global Exo
%% Gravity Compensation
fprintf('\n  Gravity Compensation')
Exo.nElements = 1
setup_temp % set most variables and plots in a SCRIPT 
[TAUsDesired,PHIs,Pos]=weightEffect(Bod,Pos);       % determine desired
[p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim
filename = 'grav_comp_latest.mat';
savdir = '/Users/partharyali/Documents/ExoNET/ExoNET/Best_Results';
save(fullfile(savdir,filename))

%% Error Augmentation
clear all; close all; clc; 
global Exo
fprintf('\n  Gravity Compensation')
Exo.nElements = 4
setup_temp % set most variables and plots in a SCRIPT 
[TAUsDesired,PHIs,Pos]=eaField(Bod);                % determine desired
[p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim
filename = 'EA_latest.mat';
savdir = '/Users/partharyali/Documents/ExoNET/ExoNET/Best_Results';
save(fullfile(savdir,filename))

%% Single Attractor
clear all; close all; clc; 
global Exo
fprintf('\n  Gravity Compensation')
Exo.nElements = 3
setup_temp % set most variables and plots in a SCRIPT 
[TAUsDesired,PHIs,Pos]=SingleAttractor(Bod);        % determine desired
[p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim
filename = 'singleattractor_latest.mat';
savdir = '/Users/partharyali/Documents/ExoNET/ExoNET/Best_Results';
save(fullfile(savdir,filename))

%% Dual Attractor
clear all; close all; clc; 
global Exo
fprintf('\n  Gravity Compensation')
Exo.nElements = 2
setup_temp % set most variables and plots in a SCRIPT 
[TAUsDesired,PHIs,Pos]=DualAttractor(Bod);          % determine desired
[p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim
filename = 'dual_attractor_latest.mat';
savdir = '/Users/partharyali/Documents/ExoNET/ExoNET/Best_Results';
save(fullfile(savdir,filename))

%% Limit Push
clear all; close all; clc; 
global Exo
fprintf('\n  Gravity Compensation')
Exo.nElements = 3
setup_temp % set most variables and plots in a SCRIPT 
[TAUsDesired,PHIs,Pos]=LimitPush(Bod);              % determine desired
[p,c,TAUs]=robustOpto(PHIs,Bod,Pos,Exo,nTries)     % ! global optim
filename = 'limitpush_latest.mat';
savdir = '/Users/partharyali/Documents/ExoNET/ExoNET/Best_Results';
save(fullfile(savdir,filename))
