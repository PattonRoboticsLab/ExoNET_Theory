clear variables
close all
clc
   
%% Choosing parameters

global L_springRest n_stackedMarionet springStiffness

n_angles = 7; %number of shoulder and elbow angles taken into account
n_stackedMarionet = 3; %number of stacked elements
L_springRest = .15; %length of the spring at rest position
springStiffness = 500;
L_UpperArm = 0.34; %length upper arm [m]
L_ForeArm = 0.29; %length fore arm [m]
Mass_TotalBody = 100; %total body mass [Kg]

[bestParam_sh, bestParam_el, averageErr_sh, averageErr_el, averageErr_percsh, averageErr_percel, Rsquared_sh, Rsquared_el] =...
    gravity_process(n_angles, L_UpperArm, L_ForeArm, Mass_TotalBody);
