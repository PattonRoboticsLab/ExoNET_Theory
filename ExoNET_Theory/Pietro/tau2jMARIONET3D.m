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

function [taus, T, Tdist]=tau2jMARIONET3D(endpoint, Actual_Pin, l0, el, stiff)
                                        
global tension 

rVect = endpoint;  % vect to rotatorPin
%norm_r = norm(rVect);
elbow = el;     wrist = endpoint;   % Elow and wrist position
elbow2wr = wrist - elbow;           % Elbow to wrist vect

Tdir  = Actual_Pin - rVect ; % Tension element vector 
Tdist = norm( Tdir ) ;       % Length, rotator2endpt
Tdir  = Tdir ./ Tdist;       % Direction vector 

T = -stiff * (l0 - Tdist);    % Uses Inline Function for Tension in Setup.m

taushoulder = cross( wrist,    T .* Tdir );  % Cross product
tauelbow    = cross( elbow2wr, T .* Tdir );  % Cross product
taus = [taushoulder; tauelbow];              % Assemble Joint 1 and 2 Torques

end % end function
    
