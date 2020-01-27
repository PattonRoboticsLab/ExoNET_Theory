% ***********************************************************************
% Plot an ellipse as a line (plot command) and returns the handle
% SYNTAX: 	hdl = ellipse(x,y,xmag,ymag,rot,num,pltcmd)
% INPUTS:	x      = x position of the center
%	    	y      = y position of the center
%		    xmag   = x-axis distance from center to edge
%		    ymag   = y-axis distance from center to edge
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

function hdl = ellipse(x,y,xmag,ymag,rot,num,pltcmd)

% SETUP
T = [cos(rot), -sin(rot); ...
	 sin(rot), cos(rot)];

% GENERATE PERIMETER POINTS
n = 0;
for i = 0:2*pi/num:2*pi
    n = n+1;
    xe(n) = xmag*cos(i);
    ye(n) = ymag*sin(i);
end

% ROTATE POINTS
for n = 1:n
    new = T*[xe(n); ye(n)];
    xe(n) = new(1);
    ye(n) = new(2);
end

% PLOT
if isstr(pltcmd)
    hdl = plot(x+xe,y+ye,pltcmd);
else
    hdl = patch(x+xe,y+ye,pltcmd,'EdgeColor','none');
end

end