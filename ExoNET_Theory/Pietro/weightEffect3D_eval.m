function [TauSh_tot, Force3D] = weightEffect3D_eval(Bod,Pos,Rot_matrix, Angles, showIt) %add eqWrF

global ProjectName 

if ~exist('showIt','var'); showIt=0; end % default
ProjectName='GravityCompensatingField';
title(ProjectName);
fprintf('\n - %s : - \n',ProjectName)

%% weights
g= 9.81;      % gravity constant
hand_weight       = (0.61/100)*Bod.M*g;                 %0.226796*g;%(0.61/100)*Bod.M*g;   % from winter's book
foreArm_weight    = (1.62/100)*Bod.M*g;                 %0.408233*g;%(1.62/100)*Bod.M*g;
forearmHandWeight = foreArm_weight+hand_weight;         % sum 'cause they're 1`

upperArm_weight   = (2.71/100)*Bod.M*g;                 %(0.453592+0.317515)*g;            %(2.71/100)*Bod.M*g;

Bod.weights       = [upperArm_weight, foreArm_weight, hand_weight]; %use a variable to attach weights of the various parts of the body

%% Torque max taken from paper 24.1+- 13.2
Torque_maxabd=27.3; % Worst case, I can reach 50% of Torque_maxabd
perc_torque=0.5; % Worst case 
%% Evaluation of the torque in 2D, taking only torque on y
for i = 1:size( Pos.EvalWr, 1 ) % Torques from weights: loop each configuration, Pos.wr since the wrist is moving in each configuration
    
    % Evaluate the torques force on shoulder and elbow joint in 2D without lifting
    % the elbow in the z direction
    TauSh_weight = cross( Pos.EvalCM1, [0, 0, -upperArm_weight]) + cross( Pos.EvalCM2(i, :), [0, 0, -forearmHandWeight]); % Evaluate the torque acting on the shoulder
    TauEl_weight = cross( Pos.EvalCM2(i, :) - Pos.EvalEl, [0, 0, -forearmHandWeight]);  % Torque force acting on the elbow

    TauEl_tot(i, :) = - TauEl_weight;    % Assign torque elbow to the variable
    TauSh_tot(i, :) = - (TauSh_weight + Torque_maxabd * [-1,1,0] * perc_torque .* [ cos(Angles.horizontal_adduction_angle) , sin(Angles.horizontal_adduction_angle) , 0]);    

end

[Force3D]=plotVectField3D_eval(Bod, Pos, TauSh_tot, TauEl_tot, Rot_matrix, Angles,  'r');

end