%% Attractor Field
function [TAUs_1,PHIs,Pos]=AttractorField(Bod);  

global F1 TAUs_1

%% Load Data
data = load('AttractorFieldData.txt');
%x01=[.5 -.1];
x01=[.3 -.1 ];
x02 = [.4 .1];
x=data(:,1:2);

%% Define Mean and Sigma of Gaussian Distribution
mu = [0.05 0.05]; % max force at this distance from center
Sigma = [1 0; 0 1]; % variance (sigma) for the Gaussian

%% Calculate Multivariate Gaussian
F = mvnpdf([x(:,1) x(:,2)],mu,Sigma);
%F = F./norm(F)

%% Initialize Force Matrix (F1(i,1) = Fx and F1(i,2) = Fy)
F1 = zeros(size(x));
for i = 1:size(x,1)
    r(i,:) = x01-x(i,:);%x(i,:)-x01;
    %r1(i,:) = x02 - x(i,:);
    %F = myGaussian(m,s,r);
    F1(i,1) = F(i).*r(i,1)+F(i).*r1(i,1);
    F1(i,2) = F(i).*r(i,2)+F(i).*r1(i,2);
    %F2(i,1) = F(i).*r(i,1);
    %F2(i,2) = F(i).*r(i,2);
    %F1(i,:) = F1(i,:)./norm(F1(i,:));
    
end
%F1 = F1+F2;
%quiver(x(:,1),x(:,2),F1(:,1),F1(:,2))

%% Connect to Main

%plot(x(:,1),x(:,2),'.','color',.8*[1 1 1]); % plot positions grey
PHIs=inverseKin(x,Bod.L); % 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations


for i=1:size(x,1), TAUs(i,:)=((jacobian(PHIs(i,:),Bod.L))*F1(i,:)'); end; %  tau=JT*F

end
 


































































