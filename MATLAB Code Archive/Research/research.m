 %%Function 1: main_tommaso_newV17.m
clear all
close all
clc

%Desired Torque (Using Fake Data)
global phi tau_d n torque err L L_upper spring y fig1 fig2 mean_err_old

%Load Data
data = load('Desired_torque_multiStable.txt'); %elbow.mat
phi = (data(:,1)); %phi units in rad
tau_d = (save(:,3;); %?? %(:,3); 

%Number of Desired MARIONETs and other constants
n=5;
L=0.26; %units in m 26 cm
L_upper = 0.36;

%functions
y=@(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(P(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); %arm for the force
spring = @(p,phi) L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi); %spring length
y=(L*p(2)*sin(p(1)-phi))./sqrt(P(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)));



fmin




%%Function 2: minimize_cost.m
%%Function 3: draw_marionet_handles.m
%%Function 4: draw_marionet_update.m
%%Function 5: draw_system.m