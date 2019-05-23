% %% Limit Push Field
function [TAUs_1,PHIs,Pos]=LimitPush(Bod);  

global TAUs_1 F x

x=[ textract('limitpush.txt','x') textract('limitpush.txt','y')]
F=[ textract('limitpush.txt','Fx') textract('limitpush.txt','Fy')]
F = F./2;

plot(x(:,1),x(:,2),'.','color',.8*[1 1 1]); % plot positions grey
PHIs=inverseKin(x,Bod.L) % 


Pos=forwardKin(PHIs,Bod)   % positions assoc w/ these angle combinations

TAUs_1 = zeros(size(x));
for i=1:size(x,1), TAUs_1(i,:)=((jacobian(PHIs(i,:),Bod.L)')*F(i,:)'); 
end; %  tau=JT*F





 end