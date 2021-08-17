clear all
close all
clc

global H phi desired_torque springStiffness L_springRest cost %some variables are global because of the way the code is done

%% Load data
Data=load('Desired_torque_fake2.txt');    %loading data
phi= Data(:,1);    %angles [rad]
desired_torque = Data(:, 2); %desired torque

%% Choosing the different parameters
choosingJoint = 0;      %where to put the device? 0: elbow joint; 1: shoulder joint.
animations = 1;         %do you want animations and figures? 0: no; 1: yes.
n_stackedMarionet = 5;  %number of stacked Marionets?
L_upperArm = .35;       %length of the upper arm? [m]
L_foreArm = .26;        %length of the fore arm? [m]
L_springRest = .45;     %length of the springs at rest position? [m]
springStiffness = 500; %spring stiffness?


%% setup plot
%plot(phi,desired_torque,'r:','linewidth',4);
%hold on;
%H=plot(1,1,'b','linewidth',3);
xlabel(' joint angle (\phi)');
ylabel('Torque (N*m)');

%% Minimization process

[optimal_param, average_err] = minimization_process(n_stackedMarionet, L_upperArm, L_foreArm, choosingJoint, animations);
