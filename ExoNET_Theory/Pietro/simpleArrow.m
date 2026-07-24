% simpleArrow:  draw arrow in 2D
%************** MATLAB "M" function  *************
% SYNTAX:     lineSeq=simpleArrow(aStart,aEnd,aColor,linWid)
% INPUTS:     aStart      1 by 2 or 1 by 3 startpoint vector
%             aEnd        similar endpoint vector arrowhead(s)
%             aColor      (optional) color spec (enter zero for no plot)
%             lineWid     (optional)  spec  -  thickness of lines
% REVISIONS:  2/8/2000    (patton) INITIATED
%             2018-Jan-14 (patton) fixed comments, added taper
%             2021-Jan-1  (patton) add loop for multiples (in rows) & 3d
%             2025-Jul-29  (patton) debug & more for multiples 
%             2025-Oct-6   (Patton) BETTER 3d  
%~~~~~~~~~~~~~~~~~~~~~ Begin : ~~~~~~~~~~~~~~~~~~~~~~~~

function lineSeq=simpleArrow(aStart,aEnd,aColor,linWid)
if ~exist('aColor','var')||isempty(aColor), aColor='r'; end;% if not passed
if ~exist('linWid','var')||isempty(linWid), linWid=1; end;  % if not passed

q=.15;      % length of arrowhead as a fraction of arrow length
c=.08;      % width of arrowhead as a fraction of arrow length
D=size(aStart,2);
[N,nDims]=size(aStart); % determine dimensionality and number of arrows

if linWid<1, return; end % skip plotting fprintf(' No plot simpleArrow.'); 

for i=1:N % loop for each row (arrow)
  
  S=aStart(i,:);
  E=aEnd(i,:);
  v=aEnd(i,:)-aStart(i,:);

  if nDims==2, v(3)=0; S(3)=0; E(3)=0; end  % add third dimension

  mag=norm(v);
  if mag==0, break; end           % do nothinig if no length
  v1=v/mag;                       % unit vect -  normalize
%   p1=cross([v 0],[0 0 1]);      % unit perpendicular (must make it 3d)
%   p1(3)=[]; p1=p1/norm(p1);     % clip back to 2D & normalize

  p1=cross(v,[0 0 1]);            % perpendicular (must be 3d)
  p1=p1/norm(p1);                 % normalize for direction vect

  p2=cross(v,p1);                 % other perpendicular (for 3d)
  p2=p2/norm(p2);                 % normalize for direction vect

  % points:
  aBreak=S+(1-q)*mag*v1;          % arrow head start point
  aBreakPlus=aBreak+.04*mag*v1;   % for the tapers
  Lside=aBreak+.5*c*mag*p1;       %
  Rside=aBreak-.5*c*mag*p1;       %

  Uside=aBreak+.5*c*mag*p2;       %
  Dside=aBreak-.5*c*mag*p2;       % 
  
  % assemble line Sequence of positions
  lineSeq=[  S                ...
            ;aBreakPlus       ...
            ;Lside            ...
            ;E        ...
            ;Rside            ...
            ;aBreakPlus       ...
    ];
    
  if nDims==3, % add orthoglnal arrowheads if 3d
    lineSeq2=[ S                ...
              ;aBreakPlus       ...
              ;Uside            ...
              ;E        ...
              ;Dside            ...
              ;aBreakPlus       ...
    ];
  end  % if 3d
  
  if aColor,
    if nDims==2, 
      % plot(lineSeq(:,1),lineSeq(:,2),'color',aColor,'Linewidth',linWid);
      patch(lineSeq(:,1),lineSeq(:,2),aColor,'EdgeColor',aColor,'Linewidth',linWid);
    elseif nDims==3,
      %plot3(lineSeq(:,1),lineSeq(:,2),lineSeq(:,3),'color',aColor,'Linewidth',linWid);
      patch(lineSeq(:,1),lineSeq(:,2),lineSeq(:,3),aColor,'EdgeColor',aColor,'Linewidth',linWid);
      patch(lineSeq2(:,1),lineSeq2(:,2),lineSeq2(:,3),aColor,'EdgeColor',aColor,'Linewidth',linWid);
    end
  end
  
end

if nargout > 0
    lineSeq = plot3(NaN, NaN, NaN, '-', 'Color', aColor, 'LineWidth', linWid);
end

end