

global phi desired_torque springStiffness L_springRest %some variables are global because of the way the code is done

%% Load data
g = 9.8;
forearm_weight = 2*g;
Data=load('elbow_flexion_momentarmdata.txt');    %loading data
phi= Data(:,1);    %angles [rad]
%desired_torque = (Data(:,4).*forearm_weight)+(Data(:,3)*forearm_weight)+(Data(:,2)*forearm_weight); %desired torque
desired_torque = Data(:,2)*forearm_weight;
%% Choosing the different parameters

choosingJoint =0;      %where to put the device? 0: elbow joint; 1: shoulder joint.
animations = 1;         %do you want animations and figures? 0: no; 1: yes.
n_stackedMarionet = 3;  %number of stacked Marionets?
L_upperArm = .35;       %length of the upper arm? [m]
L_foreArm = .26;        %length of the fore arm? [m]
L_springRest = .1;     %length of the springs at rest position? [m]
springStiffness = 20; %spring stiffness?

%% Minimization process
%2 Param

[optimal_param, average_err] = minimization_process(n_stackedMarionet, L_upperArm, L_foreArm, choosingJoint, animations);

%Cost Visualization


%4 Param

%[paramStruct, average_err, average_perc] = minimizationProcess_4param(n_stackedMarionet, L_upperArm,...
    %L_foreArm, choosingJoint, animations);

