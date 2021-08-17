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


function [tau, T, Tdist]=tauMARIONET_mod(phi,L,r0,theta0,r1,theta1,L0)
global tension
global T
%% L doesn't affect T
%LVect=[L*cos(phi)   L*sin(phi)    0];     % position vector of endpoint %
%norm_L = norm(LVect);
%%

r0Vect=[r0*cos(theta0) r0*sin(theta0)  0];     % position vector of rotatorHub-origin
r1Vect=[r1*cos(theta1+phi) r1*sin(theta1+phi)  0];     % position vector of rotatorHub-insertion
Tdir=r1Vect-r0Vect;                            % vector of tension element
Tdist=norm(Tdir);                              % magnitude:length, rotator2endpt
Tdir=Tdir./Tdist;                              % tension direction vector 

T = tension(L0,Tdist);                    % Uses Inline Function for Tension in Setup.m

tauVect=cross(r1Vect,T.*Tdir);             % cross product
tau=tauVect(3);                           % 3rd-dim is torque

end % end function