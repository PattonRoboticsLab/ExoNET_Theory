% eaField: caluate a special force field for horizontal reach
% SYNTAX: tau=eaField(Bod);                     % set torques2cancelGrav
% ~~~ BEGIN ~~~

function [TAUs,PHIs,Pos]=eaField(Bod);                     % set torques2cancelGrav
global ProjectName
ProjectName='ErrorAugmentationField';
fprintf('\n - %s : - \n',ProjectName)
x=[ textract('EAFieldData.txd','x') textract('EAFieldData.txd','y')];
F=[ textract('EAFieldData.txd','Fx') textract('EAFieldData.txd','Fy')];

PHIs=inverseKin(x,Bod.L); % 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations

for i=1:size(x,1), TAUs(i,:)=((jacobian(PHIs(i,:),Bod.L)')*F(i,:)'); end; % tau=JT*F



