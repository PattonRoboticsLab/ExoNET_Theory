% Plotting function for extracting angle vs Torque of upper appendage


function [Phi,Tau] = ExoNET_TorqueAngle(L,R,theta,k,x0,x)
%% Establish Variables
theta  = theta * pi / 180 ;                                                % Angle of attachment point of elastic element converted to [ rad ]
phi    = ( 0:1:180 ) * pi / 180 ;                                          % Angles of positions of forearm relative to humerus converted to [ rad ]
                                         
%% Equations 

Ro     = ( L .* R .* sind( theta - phi ) ) ./ ...
         ( sqrt( R.^2 + L.^2 - 2 .* R .* L .* cos( theta - phi ) ) ) ;     % Equation to calculate moment arm of single joint ExoNET [ m ]
F      = - k * ( x - x0 ) ;                                                % Hookes Law of spring element with resting length x0 [ N ]
Tau    = F .* Ro ;                                                         % Equation for torque [ N m ]
Phi    = phi * 180 / pi ;                                                  % Convert phi back to degrees for plotting [degrees]