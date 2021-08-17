clear all
close all
clc

%% Specifications
global L_springRest springStiffness nTwo_stackedMarionet nEl_stackedMarionet nSh_stackedMarionet

L_UpperArm = .35;
L_ForeArm = .26;
n_angles = 7; %number of shoulder and elbow angles taken into account
nTwo_stackedMarionet = 3; %number of stacked elements
nEl_stackedMarionet = 3;
nSh_stackedMarionet = 3;
L_springRest = .1; %length of the spring at rest position
springStiffness = 100;
Mass_TotalBody = 70;

[bestParam, averageErr_el, averageErr_sh, averagePerc_el, averagePerc_sh, Rsquared_el, Rsquared_sh] =...
    gravityProcess_mixedDevice(n_angles, L_UpperArm, L_ForeArm, Mass_TotalBody);