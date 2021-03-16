%% Torques created by a 2-joints 

% vect = vector
% dir = direction
% dist = distance 

function [tau1,tau2,F,t_dist] = twojoints(phi1,phi2,l1,l2,r,theta,k,x0)

R = [r*cosd(theta)  r*sind(theta)  0];     % R vector        
L1 = [l1*cosd(phi1) l1*sind(phi1)  0];    % L1 (elbow-writst) vector
L2 = [l2*cosd(phi2) l2*sind(phi2) 0];     % L2 (elbow-writst-hand) vector
L=L1+L2;                                 % L vector 
t_direz = R-L;                             % device vector
t_dist = norm(t_direz);                    % length of device
t_dir = t_direz./t_dist;                   % direction of tension

F=k*(t_dist-x0);                          % force/tension given 
tau_1 = cross(L,F.*t_dir);                % torque generated on L1+L2
tau_2 = cross(L2,F.*t_dir);               % torque generated on L2
tau1 = tau_1(3);                 % only 3rd direction is torque
tau2= tau_2(3);
end