% Plotting function for extracting angle vs Torque of upper appendage


function [Phi,Tau] = Arm1_TorqueAngle(L,m)
%% Input Variables of Forearm
w                = m * 9.81 ;                              % Convert body mass into weight [N]
massPerc_forearm = 1.70 * 0.01 ;                           % percentage of body mass of forearm [%]
segPer_hum       = 43.20 * 0.01 ;                          % percentage of length of forearm to center of gravity [%]
w_forearm        = w * massPerc_forearm ;                  % weight of the forearm based on overall body weight [N]
l_cog            = L * segPer_hum ;                        % distance of center of gravity from proximal end of forearm [m]
%% Re-Label Variables for Torque Calculation
phi              = ( 0:1:180 ) * pi / 180 ; 
r                = l_cog ;                                 % Moment arm of force exerted by weight of forearm [m]
F                = w * cos ( phi ) ;                       % Force calculation for component of weight perpendicular to mmoment arm [ N ]
%% Calculate Torque
Tau              = F .* r ;                                % Calculate torque from weight and length of forearm [ N m ]
Phi              = phi * 180 / pi ;                        % Convert Phi back into degrees for plottin [ Degrees ]