% Danny McDermed - March 2021
% Plotting function for extracting angle vs Torque of upper appendage - 2
% joint
% Input theta in degrees 

function [Tau_Exo1,Tau_Exo2,F,t_dist] = ExoNET2_TorqueAngle(phi1,phi2,l1,l2,r,theta,k,x0)
R         = [ r*cos(theta) r*sin(theta) 0 ] ;     % Vector for moment arm       
L1        = [ l1*cos(phi1) l1*sin(phi1) 0 ] ;     % Vector from elbow to wrist vector
L2        = [ l2*cos(phi2) l2*sin(phi2) 0 ] ;     % Vector from elbow to the wrist and hand
L         = L1 + L2 ;                               % Add lengths of appendage 
t_direz   = R - L ;                                 % Vector for length of device
t_dist    = norm(t_direz) ;                         % Return norm of device length
t_dir     = t_direz./t_dist ;                       % Generate direction of tension

F         = k*(t_dist-x0) ;                         % The force within the spring 
Tau_Exo1  = cross(L,F.*t_dir) ;                     % Torque on L1+L2
Tau_Exo2  = cross(L2,F.*t_dir) ;                    % Torque on L2
Tau_Exo1  = Tau_Exo1(3) ;                           
Tau_Exo2  = Tau_Exo2(3) ;
end
                                   

