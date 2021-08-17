% simpleArrow:  draw arrow in 2D
%************** MATLAB "M" function  *************
% SYNTAX:     lineSeq=simpleArrow(aStart,aEnd,aColor,linWid)
% INPUTS:     aStart      1 by 2 startpoint
%             aEnd        1 by 2 endpoint
%             aColor      (optional) color spec (enter zero for no plot)
%             lineWid     (optional)  spec  -  thickness of lines
% REVISIONS:  2/8/2000    (patton) INITIATED
%             2018-Jan-14 (patton) fixed comments, added taper
%~~~~~~~~~~~~~~~~~~~~~ Begin : ~~~~~~~~~~~~~~~~~~~~~~~~

function lineSeq=simpleArrow(aStart,aEnd,aColor,linWid)
if ~exist('aColor','var')||isempty(aColor), aColor='r'; end;% if not passed
if ~exist('linWid','var')||isempty(linWid), linWid=3; end;  % if not passed

q=.15;      % length of arrowhead as a fraction of arrow length
c=.1;       % width of arrowhead as a fraction of arrow length
  
v=aEnd-aStart; 
mag=norm(v); 
if mag==0, return; end        % do nothinig if no length
v1=v/mag;                     % unit vect -  normalize
p1=cross([v 0],[0 0 1]);      % unit perpendicular (must make it 3d)
p1(3)=[]; p1=p1/norm(p1);     % clip back to 2D & normalize

% points:
aBreak=aStart+(1-q)*mag*v1;   % arrow head start point 
aBreakPlus=aBreak+.04*mag*v1; % for the tapers
Lside=aBreak+.5*c*mag*p1;     % 
Rside=aBreak-.5*c*mag*p1;     %

% assemble line Sequence
lineSeq=[     aStart      ...
             ;aBreakPlus  ...
             ;Lside       ...
             ;aEnd        ...
             ;Rside       ...
             ;aBreakPlus  ...             
             ];
           
if aColor,             
  plot(lineSeq(:,1),lineSeq(:,2),'color',aColor,'Linewidth',linWid);
end
