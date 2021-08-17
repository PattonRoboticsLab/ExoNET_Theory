% %% Limit Push Field
function [TAUs,PHIs,Pos]=LimitPush(Bod);  

global ProjectName
ProjectName='LimitPush';
title(ProjectName);

%%  define field
x=[ textract('limitpush.txt','x') textract('limitpush.txt','y')];
F=[ textract('limitpush.txt','Fx') textract('limitpush.txt','Fy')];
F = F./2;
plot(x(:,1),x(:,2),'.','color',.8*[1 1 1]); % plot positions grey

PHIs=inverseKin(x,Bod.L);             % convert to angles
Pos=forwardKin(PHIs,Bod);             % positions assoc w/ these angles

%% convert to torques
TAUs = zeros(size(x)); 
for i=1:size(x,1), 
  TAUs(i,:)=((jacobian(PHIs(i,:),Bod.L)')*F(i,:)'); %  tau=JT*F
end; 

 end