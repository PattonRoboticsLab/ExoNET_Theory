clear all 
close all
clc

global H phi desired_torque stiffness L0 cost

%% load data
data = load('Desired_torque_fake2.txt');
phi = data(:,1);
desired_torque = data(:,2); 

%% Choosing the Different Parameters
choosingJoint = 0; %Choose Joint to Put Device on 0: @ elbow, 1: @ shoulder
n_stacks = 5;
animations = 1;
L_upperArm = 0.35;
L_foreArm = 0.26;
L0 = 0.45;
stiffness = 500;

%% Setup Plot
xlabel('Joint Angle (\phi)');
ylabel('Torque (Nm)');

%% Begin Minimization
[optimal_parameters, average_err] = optimize(n_stacks, L_upperArm, L_foreArm, choosingJoint, animations);