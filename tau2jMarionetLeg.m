% ***********************************************************************
% Calculate the torques created by a 2-joint MARIONET
% ***********************************************************************

function [taus,T,Tdist] = tau2jMarionetLeg(phis,Ls,r,theta,L0)

global TENSION

rVect = [r*sind(theta)  -r*cosd(theta)  0];            % R vector
knee = [Ls(1)*sind(phis(1))  -Ls(1)*cosd(phis(1))  0]; % knee position
ankle = [knee(1) + Ls(2)*sind(phis(1)-phis(2)), ...    % ankle position
         knee(2) - Ls(2)*cosd(phis(1)-phis(2)), ...
         0];
knee2ankle = ankle - knee; % knee-ankle vector
Tdir = rVect - ankle;      % MARIONET vector
Tdist = norm(Tdir);        % length of the MARIONET
Tdir = Tdir./Tdist;        % direction of the Tension exerted by the MARIONET
T = TENSION(L0,Tdist);     % magnitude of the Tension exerted by the MARIONET
tau1 = cross(ankle,T.*Tdir);      % cross product between the ankle position vector and the Tension
tau2 = cross(knee2ankle,T.*Tdir); % cross product between the knee-ankle position vector and the Tension
taus = [tau1(3) tau2(3)];         % the 3rd dimension is the torque

end