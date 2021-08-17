% Marionet torque element calculation
% Patton 2019-Jan-27 splitting this off from Tommaso's code
% making thisa vector algebra approach. 
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


function tau=tauMARIONET(phi,L,r,theta,L0)

global tension

lVect=[L*cos(phi)   L*sin(phi)    0];     % position vector of endpoint
rVect=[r*cos(theta) L*sin(theta)  0];     % position vector of rotatorHub
Tdir=rVect-lVect;                         % vector of tension element
Tdist=norm(Tdir);                         % magnitude:length, rotator2endpt
Tdir=Tdir./Tdist;                         % tension direction vector 
T=tension(L0,Tdist);                      % map stretch2tension 
tauVect=cross(lVect,T.*Tdir);             % cross product
tau=tauVect(3);                           % 3rd-dim is torque

end % end function
    
