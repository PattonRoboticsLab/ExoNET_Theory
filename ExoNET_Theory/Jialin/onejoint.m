%% Torque for 1 joint (ex. elbow-wrist)

% vect = vector
% dir = direction
% dist = distance 

function [tau,F,t_dist] = onejoint(phi,l,r,theta)

syms r l theta phi 
F=10; % force/tension is given 

R = [r*cos(theta)  r*sin(theta)  0];     % R vector        
L = [l*cos(phi)  l*sin(phi)  0]          % elbow-writst vector
t_dir = R - L;                           % device vector
t_dist = norm(t_dir);                    % length of device
t_dir = t_dir./t_dist;                   % direction of tension 

tau = cross(R,F.*t_dir);                 % torque 
tau = tau(3);                            % only 3rd direction

end
