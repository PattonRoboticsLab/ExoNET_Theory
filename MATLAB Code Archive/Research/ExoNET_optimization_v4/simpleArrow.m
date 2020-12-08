% simpleArrow: draw an arrow in 2D
%************** MATLAB "M" function  *************
% SYNTAX:     lineSequence=simpleArrow(aStart,aEnd,aColor,aLinewidth)
% INPUTS:     aStart      1 by 2 startpoint
%             aEnd        1 by 2 endpoint
%             aColor      (optional) color spec for plot (enter zero for no plot)
%             aLinewidth  
% REVISIONS:  2018-Jan-14 (patton) fixed comments, added taper
%             2/8/2000  (patton) INITIATED
%~~~~~~~~~~~~~~~~~~~~~ Begin : ~~~~~~~~~~~~~~~~~~~~~~~~

function lineSequence=simpleArrow(aStart,aEnd,aColor,aLinewidth)
if ~exist('aColor')|isempty(aColor), aColor='r', end              % if not passed
if ~exist('aLinewidth')|isempty(aLinewidth), aLinewidth=1, end    % if not passed

q=.12;     % length of arrowhead as a fraction of arrow length
c=.1;    % width of arrowhead as a fraction of arrow length
  
v=aEnd-aStart;
if norm(v)==0, return; end
aLength=norm(v);
v1=v/aLength; % unit vect
p1=cross([v 0],[0 0 1]); % unit perpendicular (must make it 3d)
p1(3)=[]; % clip back to 2D
p1=p1/norm(p1);

% points:
aBreak=aStart+(1-q)*aLength*v1;
aBreakPlus=aBreak+.04*aLength*v1;   % for the tapers
Lside=aBreak+.5*c*aLength*p1;
Rside=aBreak-.5*c*aLength*p1;

% assemble line Sequence
lineSequence=[aStart  ...
             ;aBreakPlus  ...
             ;Lside   ...
             ;aEnd    ...
             ;Rside   ...
             ;aBreakPlus  ...             
             ];
           
if aColor,             
  plot(lineSequence(:,1),lineSequence(:,2),'color',aColor,'Linewidth',aLinewidth);
end
