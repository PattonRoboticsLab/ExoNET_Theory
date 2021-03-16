%% Torque for 1 joint (ex. elbow-wrist)

% vect = vector
% dir = direction
% dist = distance 

function [R,L,tau] = singlejoint(phi,l,r,theta,k,x0,x)

R = [r*cos(theta)  r*sin(theta)  0];     % R vector        
L = [l*cos(phi)  l*sin(phi)  0];         % elbow-writst vector
t_dir = R-L;                             % device vector
t_dist = norm(t_dir);                    % length of device
t_dir = t_dir./t_dist;                   % direction of tension 

F=-k*(x-x0);                             % force/tension given 
tauV = cross(R,F.*t_dir);                 % torque vector 
tau = tauV(3);                            % only 3rd direction

end
