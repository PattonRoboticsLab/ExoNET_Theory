% drawnField: drawn endpoint force field for horizontal reach
% SYNTAX: [TAUs,PHIs,Pos]=drawnField(Bod);      % set torques
% ~~~ BEGIN ~~~

function [TAUs,PHIs,Pos]=drawnField(Bod);   

fsf=100;         % scale factor for forces
aColor=.8*[1 1 1];

% Draw Field
nDesiredVectors=input('Number of desired force vectors: '); 
fprintf('\nYou draw your own %d vectors: \n', nDesiredVectors)
h=['x' char(9) 'y' char(9) 'Fx' char(9) 'Fy' char(9)]; d=[];
for i=1:nDesiredVectors
  [x,y]=ginput(2); simpleArrow([x(1) y(1)], [x(2) y(2)],aColor);
  d=[d; [x(1) y(1) fsf*(x(2)-x(1)) fsf*(y(2)-y(1))]];
end
F=[d(:,3) d(:,4)]; % pull out force vectors

plot(d(:,1),d(:,2),'.','color',aColor); % plot positions grey

PHIs=inverseKin(d(:,1:2),Bod.L);
Pos=forwardKin(PHIs,Bod);   % setup pose aspects assoc w/these angle combos

% tau=J'*F
for i=1:size(PHIs,1), TAUs(i,:)=((jacobian(PHIs(i,:),Bod.L)')*F(i,:)'); end;

mat2txt(['drawnField.txd'],h,d);

