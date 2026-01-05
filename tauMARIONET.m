% Marionet torque element calculation
% making thisa vector algebra approach. 
% VERSIONS:  2019-Jan-27 Patton, splitting this off from Tommaso's code
%            2019-Feb-10 Patton fixing bug in r vect (second component)
%            2025-Oct-16 Patton more outputs: force (T) & length (Tdist)
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


function [tau,Tdist,T]=tauMARIONET(phi,L,r,theta,L0,plotIt)
%global tension
  
lVect=[L*cos(phi)   L*sin(phi)    0];     % position vector of endpoint
rVect=[r*cos(theta) r*sin(theta)  0];     % position vector of rotatorHub
Tdir=rVect-lVect;                         % vector of tension element
Tdist=norm(Tdir);                         % magnitude:length, rotator2endpt
Tdir=Tdir./Tdist;                         % tension direction vector 
T=forceElement(L0,Tdist);                  % map stretch2tension 
tauVect=cross(lVect,T.*Tdir);             % cross product
tau=tauVect(3);                           % 3rd-dim is torque

if plotIt, plot(Tdist,-T,'.','markersize',10,'Color',plotIt); end      % 

end % end function
