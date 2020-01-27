% ***********************************************************************
% Calculate the torques created by one Marionet on 2 joints
% ***********************************************************************

function taus = tau2jMarionet(phis,Ls,r,theta,L0)

global TENSION

rVect = [r*sind(theta)  -r*cosd(theta)  0];            % x,y,z components of the R vector
knee = [Ls(1)*sind(phis(1))  -Ls(1)*cosd(phis(1))  0]; % x,y,z coordinates of the knee
ankle = [knee(1) + Ls(2)*sind(phis(1)-phis(2)), ...    % x,y,z coordinates of the ankle
         knee(2) - Ls(2)*cosd(phis(1)-phis(2)), ...
         0];
knee2ankle = ankle - knee; % x,y,z components of the leg vector
Tdir = rVect - ankle;      % x,y,z components of the Marionet
Tdist = norm(Tdir);        % length of the Marionet
Tdir = Tdir./Tdist;        % direction of the tension exerted by the Marionet
T = TENSION(L0,Tdist);     % magnitude of the tension exerted by the Marionet
tau1 = cross(ankle,T.*Tdir);      % cross product between the position vector and the tension
tau2 = cross(knee2ankle,T.*Tdir); % cross product between the position vector and the tension
taus = [tau1(3) tau2(3)];         % the 3rd dimension is the torque

end