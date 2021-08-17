clear all
close all
clc

%% Specifications
global L_springRest n_stackedMarionet springStiffness

L_UpperArm = .35;
L_ForeArm = .26;
n_angles = 7; %number of shoulder and elbow angles taken into account
n_stackedMarionet = 5; %number of stacked elements
L_springRest = .5; %length of the spring at rest position
springStiffness = 500;
Mass_TotalBody = 70;

[bestParam, averageErr_el, averageErr_sh, averagePerc_el, averagePerc_sh, Rsquared_el, Rsquared_sh] = gravityProcess_twoJoints(n_angles, L_UpperArm, L_ForeArm, Mass_TotalBody);