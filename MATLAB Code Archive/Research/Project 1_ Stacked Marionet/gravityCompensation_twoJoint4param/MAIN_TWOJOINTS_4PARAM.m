clear all
close all
clc

%% Specifications
global n_stackedMarionet

L_UpperArm = .34;
L_ForeArm = .29;
n_angles = 7; %number of shoulder and elbow angles taken into account
n_stackedMarionet = 6; %number of stacked elements
Mass_TotalBody = 100;

[bestParam, averageErr_el, averageErr_sh, averagePerc_el, averagePerc_sh, Rsquared_el, Rsquared_sh] =...
    gravityProcess_twoJoints4param(n_angles, L_UpperArm, L_ForeArm, Mass_TotalBody);