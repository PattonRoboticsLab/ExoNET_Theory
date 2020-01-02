%% Attractor Field
function [TAUs,PHIs,Pos]=AttractorField2(Bod);  
%  clc
%  clear all
% close all
global F1
%% Load Data
data = load('AttractorFieldData.txt');
s =.08;% variance (sigma) for the Gaussian
m=0.05; % max force at this distnance from center
%x01=[.5 -.1];
x01=[.3 -.1 ];
x=data(:,1:2);


[X Y] = meshgrid(x(:,1),x(:,2));
mu = [.05 .05];
Sigma = [.08 0; 0 .08];
F = mvnpdf([x(:,1) x(:,2)],mu,Sigma);
F1 = zeros(size(x));
for i = 1:size(x,1)
    r(i,:) = x01-x(i,:);%x(i,:)-x01;
    %F = myGaussian(m,s,r);
    F1(i,1) = F(i).*r(i,1);
    F1(i,2) = F(i).*r(i,2);
    %plot(r(i,1),F1(i,1),'bo',r(i,2),F1(i,2),'ro')
    %plot(r,F1,'bo');
    %hold on
    %pause(.01)
end

%quiver(x(:,1),x(:,2),F1(:,1),F1(:,2))
% Connect to Main
%plot(transpose(x),'.','color',.8*[1 1 1]); % plot positions grey
% plot(x01(1,1),x01(1,2));
PHIs=inverseKin(x,Bod.L); % 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations


for i=1:size(x,1), TAUs(i,:)=((jacobian(PHIs(i,:),Bod.L))*F1(i,:)'); end; %  tau=JT*F

end
 


































































