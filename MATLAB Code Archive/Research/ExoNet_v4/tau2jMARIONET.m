% 2 joint Marionet torque element calculation
%  Using vector algebra. 
% VERSIONS:  2019-Jan-31 Patton Created from tauMARIONET
%            2019-Feb-10 Patton fixed (small!) L2 bug in elbow position,
%                        and (bigger!) r bug in rVect
%           
%                                                  endpoint: wr>  
%                                            .  X  
%                                      .     X  elbow2wr>
%                        < TVect  .       X   
%                           .          X              . 
%                     .             X    phi2   . 
%                .               X        . 
%           .                 X     . 
%    _   o                 X  .         
%    r  /               O
%      / theta     X    
%     /       X       
%    /   X      phi1
%   o  .  .  .  .  .  .  .  .  .  .  .  . 
%
%
% ~~ BEGIN PROGRAM: ~~



function taus=tau2jMARIONET(phis,Ls,r,theta,L0)

global tension

rVect=[r*cos(theta)       r*sin(theta)       0];  % vect to rotatorPin
elbow=[Ls(1)*cos(phis(1)) Ls(1)*sin(phis(1)) 0];  % elbow pos
wrist=[elbow(1)+Ls(2)*cos(phis(1)+phis(2)), ...   % wrist pos
       elbow(2)+Ls(2)*sin(phis(1)+phis(2)), ...
       0];
elbow2wr=wrist-elbow;                             % elbow to wrist vect
Tdir=rVect-wrist;                                 % tension element vector 
Tdist=norm(Tdir);                                 % length, rotator2endpt
Tdir=Tdir./Tdist;                                 % direction vector 
T=tension(L0,Tdist);                              % map stretch2tension 
tau1=cross(wrist,T.*Tdir);                        % cross product
tau2=cross(elbow2wr,T.*Tdir);                     % cross product
taus=[tau1(3) tau2(3)];                           % 3rd-dim is torque

end % end function
    
