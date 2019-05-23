clear all
close all
clc
 
global phi desired_torque %some variables are global because of the way the code is done
 
%% Load data
 
phi = linspace(0,3); %let's use angles from sample space 0-3
desired_torque = sin(phi)+cos(phi); %torque profile follows trajectory of sin+cos
 
%% Choosing the different parameters
 
joint = 0;      %where to put the device? 0: elbow joint; 1: shoulder joint.
animations = 1;         %do you want animations and figures? 0: no; 1: yes.
n_stackedMarionet = 5;  %number of stacked Marionets?
L_upperArm = .35;       %length of the upper arm? [m]
L_foreArm = .26;        %length of the fore arm? [m]
%% Minimization process
 
[paramStruct, average_err, average_perc] = minimizationProcess_4param(n_stackedMarionet, L_upperArm,...
    L_foreArm, choosingJoint, animations);

phi = linspace(0,3); %let's use angles from sample space 0-3
desired_torque = sin(phi)+cos(phi); %torque profile follows trajectory of sin+cos