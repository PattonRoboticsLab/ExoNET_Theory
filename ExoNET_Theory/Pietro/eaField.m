% eaField: caluate a special force field for horizontal reach
% SYNTAX: tau=eaField(Bod);     % set torques2cancelGrav
% ~~~ BEGIN ~~~

function [TAUs, PHIs, Pos] = eaField(Bod)      % set torques2cancelGrav

global ProjectName
ProjectName = 'ErrorAugmentationField';
% Create the errorfield
Bod.nAngles = 3;      % # shoulder & elbow angles 
Bod.nAngles_z = 3;    % # elevation planes
Bod.n_angles_3D = 3;  % # rotation of the elevation plane

% Input AngleEA
AngleEA = input('Type an angle from 0 to 90 degrees: ');

phi1EA = pi/180 * linspace( 290+10^-6,        350+10^-6,         Bod.nAngles );   % Phi1 see the figure above
phi2EA = pi/180 * linspace( 10+10^-6,         80+10^-6,          Bod.nAngles );   % Phi2 see the figure above
phi3EA = pi/180 * linspace( AngleEA+10^-6-45, AngleEA+10^-6+45,  Bod.nAngles_z ); % Phi3 see the figure above 3D, from rest position when relaxed to max flexion PB 

PHIs.EA = [];  % Initialize the vector of the angles for elevation

% Nested 3-loop establishes grid of phi's moving, this dictates the movement of the arm in the space
for i = 1:length(phi3EA)         % For every angle of the sagital plane 
    for j = 1:length(phi1EA)     % Flex the shoulder
        for k = 1:length(phi2EA) % Flex the elbow
            PHIs.EA = [PHIs.EA; phi1EA(j), phi2EA(k), phi3EA(i)]; % Store all the angles
        end 
    end
end

[Pos.EA, ~, ~] = forwardKin3D(PHIs, Bod, Exo, 'EA'); 

% 
% fprintf('\n - %s : - \n', ProjectName)
% x = [ textract('EAFieldData.txd','x')  textract('EAFieldData.txd','y')];

F = [ textract('EAFieldData.txd','Fx') textract('EAFieldData.txd','Fy')];

PHIs = inverseKin(x, Bod.L);  % 
Pos = forwardKin(PHIs, Bod);  % positions assoc w/ these angle combinations

for i=1:size(x,1) 
    TAUs(i,:) = ( (jacobian( PHIs(i,:), Bod.L)' ) * F(i,:)' ); 
end 

end % tau=JT*F




