% eaField: caluate a special force field for horizontal reach
% SYNTAX: tau=eaField(Bod);                     % set torques2cancelGrav
% ~~~ BEGIN ~~~

function [TAUs,PHIs,Pos]=eaField(Bod);                     % set torques2cancelGrav

f=10;
%d=hdrload('')

x=[ textract('EAFieldData.txd','x') textract('EAFieldData.txd','y')];
F=[ textract('EAFieldData.txd','Fx') textract('EAFieldData.txd','Fy')];
plot(x(:,1),x(:,2),'.','color',.6*[1 1 1]); % plot positions grey

PHIs=inverseKin(x,Bod.L); % 
Pos=findPos(PHIs,Bod);   % positions assoc w/ these angle combinations

for i=1:size(x,1), TAUs(i,:)=((jacobian(PHIs(i,:),Bod.L)')*F(i,:)'); end; % tau=JT*F



