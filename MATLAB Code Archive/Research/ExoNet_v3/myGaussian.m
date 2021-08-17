
function F = myGaussian(m,s,r)

F = (1/(s.*sqrt(2*pi)))*exp(-.5*((r-m)/s).^2);
%     F(i,1) = (1/(sqrt(2*pi)*s))*exp(-.5*(r1(i,1).^2)/(s^2));
%     F(i,2) = (1/(sqrt(2*pi)*s))*exp(-.5*(r1(i,2).^2)/(s^2));

 %F = F./norm(F);
end





% for i = 1:size(x,1)
%     r1(i,:)=x(i,:)-x01;
%     F(i,:) = r1(i,:).*F(i,1);
% 
%     %F(i,:) = (1/(s.*sqrt(2*pi)))*exp((-.5*((r1(i,:)-m)/s).^2));

%     %plot(r1(i,1),F(i,1),'bo',r1(i,2),F(i,2),'ro')
%     %hold on
%     %plot(r1(i,1),Fx(i,1),'bo',r1(i,2),Fy(i,1),'ro')
%     %hold on
%     %F(i,:) = r1(i,:)*F(i,1);
% end
