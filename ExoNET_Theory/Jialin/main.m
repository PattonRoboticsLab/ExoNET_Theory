%% Plotting torque field with respect to angle phi - 1joint 

clear all 
close all
clc

% input variables 
phi=-5:160;                                                                             % angle of rotation of forearm [deg]
theta=0:360;                                                                            % angle of rotation of elastic attach site with respect to elbow [deg]
l=0.28;                                                                                 % average length of forearm [m]
r=input('Insert distance between elbow and elastic element attachment site [m]: ')      % distance between elbow and elastic element attachment site [m]
k=input('Insert elastic coefficient[N/m]: ')                                            % coefficient of elastic element [N/m]
x=input('Insert final length of elastic element [m]: ')                                 % final length of elastic element [m]
x0=input('Insert initial length of elastic element [m]: ')                              % initial length of elastic element [m]

for i=1:166
    for j=1:361
        [R,L,tau] = singlejoint(phi(i),l,r,theta(j),k,x0,x);
        rho(i,j)=tau;
    end
end

plot(phi,rho(:,1),'-r')

