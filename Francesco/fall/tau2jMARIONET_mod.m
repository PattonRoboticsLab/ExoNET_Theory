function [taus, T, Tdist]=tau2jMARIONET_mod(phis,Ls,r0,theta0,r2,theta2,L0)

global tension 

r0Vect=[r0*cos(theta0)       r0*sin(theta0)       0];  % vect to rotatorPin
r2Vect=[r2*cos(theta2+phis(1))       r2*sin(theta2+phis(1))       0];
%norm_r = norm(rVect);
elbow=[Ls(1)*cos(phis(1)) Ls(1)*sin(phis(1)) 0];  % elbow pos
wrist=[elbow(1)+Ls(2)*cos(phis(1)+phis(2)), ...   % wrist pos
       elbow(2)+Ls(2)*sin(phis(1)+phis(2)), ...
       0];
elbow2wr=wrist-elbow;                             % elbow to wrist vect                                 % tension element vector 
Tdir=elbow+r2Vect-r0Vect;
Tdist=norm(Tdir) ;                                % length, rotator2endpt
Tdir=Tdir./Tdist;    % direction vector 

T = tension(L0,Tdist);                    % Uses Inline Function for Tension in Setup.m

tau1=cross(elbow+r2Vect,T.*Tdir);                        % cross product
tau2=cross(r2Vect,T.*Tdir);                     % cross product
taus=[tau1(3) tau2(3)];                           % Assemble Joint 1 and 2 Torques

end % end function
    
