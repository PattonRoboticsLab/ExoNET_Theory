clear all
close all
clc

%% Global Variables
%Definitions: phi - phi angles, desired_torque: customized
%torque profile, k_spring: stiffness of the spring, Lsp_rest: Length of
%the Spring at Rest

global phi desired_torque k_sp Lsp_rest 

%% Load Data
data = load('Desired_torque_fake2.txt'); %Calling the torque profile text file
phi = data(:,1); %defining theta angles, phi
desired_torque = data(:,2); %desired torque profile

%% Device Location and Figures?
joint = 0; %Device Location: 0: elbow joint, 1: shoulder joint
animations = 1; %Generate Figures and Draw System, 0 - no, 1 - yes

%% Device Given Parameters
n_stacks = 5; %number of stacked MARIONET elements
L_upper = 0.35; %Length of upper arm (Shoulder Joint - Elbow Joint) [meters]
L_fore = 0.26; %Length of Forearm (Elbow Joint - Wrist Joint) [meters]
Lsp_rest = 0.45; %Resting Length of Spring [meters]
k_sp = 1000; %stiffness of spring (k) [Units]

%% Minimization Process
[optimal_parameter,average_error] = minimization_process(n_stacks, L_upper, L_fore, joint, animations);

