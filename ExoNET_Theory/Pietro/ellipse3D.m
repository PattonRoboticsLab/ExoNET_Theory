% ***********************************************************************
% Plot an ellipse as a line (plot command) and returns the handle
% SYNTAX: 	hdl = ellipse(x,y,xmag,ymag,rot,num,pltcmd)
% INPUTS:	x      = x position of the center
%	    	y      = y position of the center
%           z      = z position of the center
%		    xmag   = x-axis distance from center to edge
%		    ymag   = y-axis distance from center to edge
%           zmag=  = z-axis distance from center to edge
%		    rot    = rotation counterclockwise (rads)
%		    num    = number of vertices (suggestion: 20)
%		    pltcmd = plotting specification
%                    if it is a string, then the outline is drawn
%                    based on pltcmd
%     		         (example: 'r+' means plot red plusses)
%		             if it is a vector of 3 RGB values, then it 
%		             shades the ellipse accroding to pltcmd 
%     		         (example: [0.5, 0.5, 0.5] means shade it gray)
% ***********************************************************************

function hdl = ellipse3D(x,y,z,xmag,ymag,zmag,pltcmd)

% SETUP
center=[x,y,z];

% Generate the parametric angles
theta = linspace(0, 2*pi, 50);  % Azimuth angle
phi = linspace(0, pi, 25);      % Polar angle

x = xmag * sin(phi') * cos(theta) + center(1);
y = ymag * sin(phi') * sin(theta) + center(2);
z = zmag * cos(phi') * ones(size(theta)) + center(3);

% Plot the 3D oval using fill3

for i = 1:length(phi)-1
    for j = 1:length(theta)-1
        % Define vertices of a small patch
        vx = [x(i, j), x(i+1, j), x(i+1, j+1), x(i, j+1)];
        vy = [y(i, j), y(i+1, j), y(i+1, j+1), y(i, j+1)];
        vz = [z(i, j), z(i+1, j), z(i+1, j+1), z(i, j+1)];
        % Fill the patch
        hdl = fill3(vx, vy, vz, pltcmd, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    end
end

end %end of the function