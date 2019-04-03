%% Attractor Field
function [TAUs_1,PHIs,Pos]=AttractorField(Bod);  

global F1 TAUs_1

%% Load Data
%data = load('AttractorFieldData.txt');
%x=data(:,1:2);
%x01=[.5 -.1];
x01=[.3 0];
x02 = [.3 -.2];

%First Attractor Field Information
a = .1:.025:.5;
b = -0.2:.025:0.2;
[A B] = meshgrid(a,b);
c=cat(2,A',B');
g = reshape(c,[],2);
x1 = g(:,1:2);

%Second Attractor Field Information
a = .25:.025:.35;
b = -.25:.025:-.15;
[A B] = meshgrid(a,b);
c=cat(2,A',B');
g = reshape(c,[],2);
x2 = g(:,1:2);

x = [x1;x2];

%% Define Mean and Sigma of Gaussian Distribution
mu = [0.05 0.05]; % max force at this distance from center
Sigma = [1 0; 0 1]; % variance (sigma) for the Gaussian

%% Calculate Multivariate Gaussian
F = mvnpdf([x(:,1) x(:,2)],mu,Sigma);
%F = F./norm(F)
Fx1 = mvnpdf([x1(:,1) x1(:,2)],mu,Sigma);
Fx2 = mvnpdf([x2(:,1) x2(:,2)],mu,Sigma);
%% Initialize Force Matrix (F1(i,1) = Fx and F1(i,2) = Fy)
F1 = zeros(size(x));
F1x1 = zeros(size(x1));
for i = 1:size(x1,1)
    r(i,:) = x01-x1(i,:);%x(i,:)-x01;
    
    F1x1(i,1) = Fx1(i).*r(i,1);%+F(i).*r1(i,1);
    F1x1(i,2) = Fx1(i).*r(i,2);%+F(i).*r1(i,2);
        
end

F1x2 = zeros(size(x2));
for i = 1:size(x2,1)
    r(i,:) = x02-x1(i,:);%x(i,:)-x01;
    %r1(i,:) = x02 - x(i,:);
    %F = myGaussian(m,s,r);
    F1x2(i,1) = Fx2(i).*r(i,1);%+F(i).*r1(i,1);
    F1x2(i,2) = Fx2(i).*r(i,2);%+F(i).*r1(i,2);
        
end

%F1 = [F1x1;F1x2];
F1 = F1x1;
x = x1;
%F1 = F1+F2;
%quiver(x(:,1),x(:,2),F1(:,1),F1(:,2))

%% Connect to Main

%plot(x(:,1),x(:,2),'.','color',.8*[1 1 1]); % plot positions grey
PHIs=inverseKin(x,Bod.L); % 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations

TAUs_1 = zeros(size(x));
for i=1:size(x,1), TAUs_1(i,:)=((jacobian(PHIs(i,:),Bod.L))*F1(i,:)'); end; %  tau=JT*F

end
