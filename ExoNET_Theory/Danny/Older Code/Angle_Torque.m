% Plotting function for extracting angle vs Torque of upper appendage


function [Phi,Tau] = Angle_Torque(l,m)
%% Input Variables of Humerus
w            = m * 9.81 ;                              % Convert body mass into weight [N]
massPerc_hum = 3.00 * 0.01 ;                           % percentage of body mass of humerus [%]
segPer_hum   = 45.00 * 0.01 ;                          % percentage of length of forearm to center of gravity [%]
w_hum        = w * massPerc_hum ;                      % weight of the humerus based on overall body weight [N]
l_cog        = l * segPer_hum ;                        % distance of center of gravity from proximal end of humerus [m]
%% Re-Label Variables for Torque Calculation
r = l_cog ; % Moment arm of force exerted by weight of humerus [m]

