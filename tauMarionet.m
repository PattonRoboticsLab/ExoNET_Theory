% ***********************************************************************
% Calculate the torque created by one Marionet
% ***********************************************************************

function tau = tauMarionet(phi,L,r,theta,L0)

global TENSION
  
lVect = [L*sind(phi)   -L*cosd(phi)    0]; % x,y,z components of the Limb vector
rVect = [r*sind(theta) -r*cosd(theta)  0]; % x,y,z components of the R vector
Tdir = rVect - lVect;                      % x,y,z components of the Marionet
Tdist = norm(Tdir);                        % length of the Marionet
Tdir = Tdir./Tdist;                        % direction of the tension exerted by the Marionet
T = TENSION(L0,Tdist);                     % magnitude of the tension exerted by the Marionet
tauVect = cross(rVect,T.*Tdir);            % cross product between the position vector R and the tension
tau = tauVect(3);                          % the 3rd dimension is the torque

end