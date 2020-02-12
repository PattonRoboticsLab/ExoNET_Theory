% ***********************************************************************************
% Draw an arrow in 2D
% SYNTAX:     lineSequence = simpleArrow(arrowStart,arrowEnd,arrowColor,lineWidth)
% INPUTS:     arrowStart   = 1 by 2 startpoint
%             arrowEnd     = 1 by 2 endpoint
%             arrowColor   = (optional) color specification (enter zero for no plot)
%             lineWidth    = (optional) specification of lines thickness
% ***********************************************************************************

function lineSequence = simpleArrow(arrowStart,arrowEnd,arrowColor,lineWidth)

if ~exist('arrowColor','var')||isempty(arrowColor)
    arrowColor = 'r';
end
if ~exist('lineWidth','var')||isempty(lineWidth)
    lineWidth = 3;
end

q = 0.15; % length of the arrowhead as a fraction of the arrow length
c = 0.1;  % width of the arrowhead as a fraction of the arrow length

v = arrowEnd - arrowStart;
magnitude = norm(v);       % magnitude of the vector
if magnitude == 0
    return                 % if the length is zero, do nothing
end
v1 = v/magnitude;          % unit vector (normalized vector)
p1 = cross([v 0],[0 0 1]); % vector perpendicular to v (is 3D)
p1(3) = [];                % to make it 2D
p1 = p1/norm(p1);          % unit vector perpendicular to v (normalized vector)

% Points
arrowBreak = arrowStart + (1-q)*magnitude*v1;    % arrow head start point
arrowBreakPlus = arrowBreak + 0.04*magnitude*v1; % for the taper
leftSide = arrowBreak + 0.5*c*magnitude*p1;
rightSide = arrowBreak - 0.5*c*magnitude*p1;

% Assemble the line sequence
lineSequence = [arrowStart; ...
                arrowBreakPlus; ...
                leftSide; ...
                arrowEnd; ...
                rightSide; ...
                arrowBreakPlus];

if arrowColor
    plot(lineSequence(:,1),lineSequence(:,2),'Color',arrowColor,'Linewidth',lineWidth)
end

end