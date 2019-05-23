clear variables
close all
clc

%% 04/04/18 
    %every force is normalized according to the overall maximum
    %force, the optimization process has 3 parameters for each stacked
    %element and, moreover, the force of the spring is =0 during
    %compression.
    %Everything happens in the "optimizedTorque_gravityField.m" function.
    
%% 04/06/18 
    %now the parameters are unique for the overall space, one set for the
    %shoulder device and one for the elbow one.
    %Everything happens in the "optimizedTorque_gravityField_overall.m" function.

%% 04/16/18
    %the torque cost calculation now is done with the least square error of
    %the torque profile with all the torques on the overall space;
    %all the function used are the ones labeled as "new".
    
%% Choosing parameters

global L_springRest n_stackedMarionet

n_angles = 10; %number of shoulder and elbow angles taken into account
n_stackedMarionet = 2; %number of stacked elements
L_springRest = .20; %length of the spring at rest position

L_UpperArm = 0.35; %length upper arm [m]
L_ForeArm = 0.26; %length fore arm [m]
Mass_TotalBody = 70; %total body mass [Kg]

[bestParam_sh, bestParam_el] = gravity_process(n_angles, L_UpperArm, L_ForeArm, Mass_TotalBody);
