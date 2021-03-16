% ***********************************************************************
% Calculate the torque created by a MARIONET
% ***********************************************************************

function [tau,T,Tdist] = tauMarionetLeg(phi,L,r,theta,L0)

global TENSION

rVect = [r*sind(theta) -r*cosd(theta)  0]; % R vector
lVect = [L*sind(phi)   -L*cosd(phi)    0]; % hip-knee or knee-ankle vector
Tdir = rVect - lVect;                      % MARIONET vector
Tdist = norm(Tdir);                        % length of the MARIONET
Tdir = Tdir./Tdist;                        % direction of the Tension exerted by the MARIONET
T = TENSION(L0,Tdist);                     % magnitude of the Tension exerted by the MARIONET
tauVect = cross(rVect,T.*Tdir);            % cross product between the R position vector and the Tension
tau = tauVect(3);                          % the 3rd dimension is the torque

end