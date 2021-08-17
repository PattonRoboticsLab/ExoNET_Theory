clc
clear all
close all

%1 Stack MARIONET
rng default
n=1; %Number of Stacks
L = .34; %Length of Arm
%% Load data

Data=load('Desired_torque_fake2.txt');    %loading data
phi= Data(:,1);    %angles [rad]
torque = Data(:, 2); %desired torque


lb = []; %Lower Bounds of Optimized Values
ub = []; %Upper Bounds of Optimized Values
fun = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); %Function defining the moment arm of the force
%p(1) = angle p(2) = radius
x0 = zeros(1,2*n); %Initial Guesses for Optimized P values
p = lsqcurvefit(fun,x0,phi,torque,lb,ub); %Least Squares Fit Optimizations
figure(1) %Plotting Phi, Torque relationship with Optimized Values
plot(phi,torque,'ko',phi,fun(p,phi),'r')
residual1 = torque-fun(p,phi);