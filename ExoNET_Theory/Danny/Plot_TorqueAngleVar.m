% Program to plot Torque generated by single element ExoNET versus angle of
% point of attachment

%% Clear Workspace, Command Window, etc.
clc 
clear all
close all
%% Input Variables
L                  = 0.3048 ;                                                                      % Length of forearm [ m ]          
R                  = 0.20 ;                                                                        % Element attachment point distance from elbow joint [ m ] 
theta              = 90 ;                                                                         % Angle of elastic element attachment point relative to humerus [ deg ]
k1                 = input(' Tested Spring Coefficient 1 = ') ;                                    % Spring Coefficient of elastic element [ N / m ]
k2                 = k1 * 2 ;                                                                      % Spring Coefficient of elastic element [ N / m ]
k3                 = k1 * 4 ;                                                                      % Spring Coefficient of elastic element [ N / m ]
k4                 = k1 * 6 ;                                                                      % Spring Coefficient of elastic element [ N / m ]
k5                 = k1 * 8 ;                                                                      % Spring Coefficient of elastic element [ N / m ]
k6                 = k1 * 10 ;                                                                     % Spring Coefficient of elastic element [ N / m ]
k7                 = k1 * 12 ;                                                                     % Spring Coefficient of elastic element [ N / m ]
k8                 = k1 * 14 ;                                                                     % Spring Coefficient of elastic element [ N / m ]
k9                 = k1 * 16 ;                                                                     % Spring Coefficient of elastic element [ N / m ]
k10                = k1 * 18 ;                                                                     % Spring Coefficient of elastic element [ N / m ]
x0                 = 0.05 ;                                                                        % Resting length of elastic element [ m ]
x                  = 0.15 ;                                                                         % Final length of elastic element [ m ]

%% Call Function to Calculate Torque
[Phi_Exo1, Tau_Exo1]   = ExoNET1_TorqueAngle(L,R,theta,k1,x0,x) ;                                  % Use function to calculate torque and extract variables for ExoNET
[Phi_Exo2, Tau_Exo2]   = ExoNET1_TorqueAngle(L,R,theta,k2,x0,x) ;                                  % Use function to calculate torque and extract variables for ExoNET
[Phi_Exo3, Tau_Exo3]   = ExoNET1_TorqueAngle(L,R,theta,k3,x0,x) ;                                  % Use function to calculate torque and extract variables for ExoNET
[Phi_Exo4, Tau_Exo4]   = ExoNET1_TorqueAngle(L,R,theta,k4,x0,x) ;                                  % Use function to calculate torque and extract variables for ExoNET
[Phi_Exo5, Tau_Exo5]   = ExoNET1_TorqueAngle(L,R,theta,k5,x0,x) ;                                  % Use function to calculate torque and extract variables for ExoNET
[Phi_Exo6, Tau_Exo6]   = ExoNET1_TorqueAngle(L,R,theta,k6,x0,x) ;                                  % Use function to calculate torque and extract variables for ExoNET
[Phi_Exo7, Tau_Exo7]   = ExoNET1_TorqueAngle(L,R,theta,k7,x0,x) ;                                  % Use function to calculate torque and extract variables for ExoNET
[Phi_Exo8, Tau_Exo8]   = ExoNET1_TorqueAngle(L,R,theta,k8,x0,x) ;                                  % Use function to calculate torque and extract variables for ExoNET
[Phi_Exo9, Tau_Exo9]   = ExoNET1_TorqueAngle(L,R,theta,k9,x0,x) ;                                  % Use function to calculate torque and extract variables for ExoNET
[Phi_Exo10, Tau_Exo10] = ExoNET1_TorqueAngle(L,R,theta,k10,x0,x) ;                                 % Use function to calculate torque and extract variables for ExoNET

%% Plot Torque vs Input Angle of Forearm Position
figure                                                                                             % Assign space for plot
plot(Phi_Exo1,Tau_Exo1) ;                                                                          % Function to plot input variables
title('Generated Torque vs Angular Position of Forearm') ;                                         % Assign title to generated plot
xlabel('Phi [Degrees]') ;                                                                          % Assign x-axis label to generated plot
ylabel('Tau [Nm]') ;                                                                               % Assign y-axis label to generated plot

hold on                                                                                            % Function to allow for other plots to be on same figure

plot(Phi_Exo2,Tau_Exo2)                                                                            % Function to plot input variables
plot(Phi_Exo3,Tau_Exo3)                                                                            % Function to plot input variables
plot(Phi_Exo4,Tau_Exo4)                                                                            % Function to plot input variables
plot(Phi_Exo5,Tau_Exo5)                                                                            % Function to plot input variables
plot(Phi_Exo6,Tau_Exo6)                                                                            % Function to plot input variables
plot(Phi_Exo7,Tau_Exo7)                                                                            % Function to plot input variables
plot(Phi_Exo8,Tau_Exo8)                                                                            % Function to plot input variables
plot(Phi_Exo9,Tau_Exo9)                                                                            % Function to plot input variables
plot(Phi_Exo10,Tau_Exo10)                                                                          % Function to plot input variables

hold off                                                                                           % Finalize plotting of different variables

% Note : Next step is to make a code that iterates this process


