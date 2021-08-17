% Marionet torque element calculation
% making thisa vector algebra approach. 
% VERSIONS:  2019-Jan-27 Patton, splitting this off from Tommaso's code
%            2019-Feb-10 Patton fixing bug in r vect (second component)
%
%                             .X
%                       .   X  
%         TVect   .      X       lVect
%           .         X
%       o          X       
%   r  /        X
%     /      X       phi
%    /   theta        
%   /  X            
%  O .  .  .  .  .  .  .  .  .  .  .  . 


function [tau, T, Tdist]=tauMARIONET(phi,L,r,theta,L0)
global tension
  
rVect=[L*cos(phi)   L*sin(phi)    0];     % position vector of endpoint
norm_r = norm(rVect);


lVect=[r*cos(theta) r*sin(theta)  0];     % position vector of rotatorHub
Tdir=lVect-rVect;%rVect-lVect;            % vector of tension element
Tdist=norm(Tdir);                         % magnitude:length, rotator2endpt
Tdir=Tdir./Tdist;                         % tension direction vector 


T = tension(L0,Tdist);                    % Uses Inline Function for Tension in Setup.m


tauVect=cross(lVect,T.*Tdir);             % cross product
tau=tauVect(3);                           % 3rd-dim is torque

end % end function
    
