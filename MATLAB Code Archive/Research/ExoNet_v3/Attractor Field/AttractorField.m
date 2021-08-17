%% Attractor Field
% clc 
% clear all 
% close all

function [TAUs,PHIs,Pos]=AttractorField(Bod);  

data = load('AttractorFieldData.txt');
% X = data(:,1);
% Y = data(:,2

% XY = [X Y];
s = .2; % variance (sigma) for the Gaussian
m=.20; % max force at this distnance from center
%x01=[.7 .3 ];
%x02=[.3 -.3 ]
x=data(:,1:2);
x01 = [mean(x(:,1)) mean(x(:,2))];


s1 = std(x(:,1));
s2 = std(x(:,2));
% Fx = 
% Fy = normpdf(x(:,2),x01(2,1),s2);

Fx = zeros(size(x));
Fy = zeros(size(x));
for i = 1:size(x,1)
    r1(i,:)=x(i,:)-x01;
    Fx(i)= normpdf(x(i,1),x01(1,1),s1);
    Fy(i) = normpdf(x(i,2),x01(1,2),s2);
    F = [Fx(i,1) Fy(i,1)];
    
    %plot(r1(i,1),Fx(i,1),'bo',r1(i,2),Fy(i,1),'ro')
    %hold on
end
F = [Fx(:,1), Fy(:,1)]

% X = .25:0.05:.45;
% Y = -.1:0.05:.1;
% XY = [X Y];

%F=zeros(size(x));


%  for i = 1:size(x,1)
%       %r(i,:) = pdist(XX,'euclidean');
%       r1(i,:)=x(i,:)-x01;
%       r3 = r1(i,1);
%       r4 = r1(i,2);
%       %[Fx, Fy] = myGaussian(m,s,r);
%       %F(i,:) = 1*exp(-.5*r1(i,1).^2-.5*(r1(i,2).^2));
%       Fx(i) = (1/(s1*sqrt(2*pi)))*exp(-r1(i,1)^2/(2*s1^2));
%       Fy(i) = (1/(s2*sqrt(2*pi)))*exp(-r1(i,2)^2/(2*s2^2));
%         
%       v1 = [r3,Fx(i,1)];
%       Fx(i) = Fx(i,1)/norm(v1);
%       v2 = [r4,Fy(i,1)];
%       Fy(i) = Fy(i,1)/norm(v2);
% %       plot(r,F,'bo')
% %       hold on 
% %       pause(.01)
%     
%       %r2=x(i,:)-x02;
% %     r1hat=r1./norm(r1); %r2hat=r2./norm(r2);
% %     F(i,:) = (1/(s.*sqrt(2*pi)))*exp((-.5*((r1(i,:)-m)./s).^2)); %+ r2hat*(1/(s.*sqrt(2*pi)))*exp(-.5*((norm(r2)-m)/s).^2) ;
% %     plot(r1,F,'o')
% %     hold on
%  end
% F = [Fx(:,1) Fy(:,1)]
%F
%F

% Connect to Main
%plot(transpose(x),'.','color',.8*[1 1 1]); % plot positions grey

PHIs=inverseKin(x,Bod.L); % 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations


for i=1:size(x,1), TAUs(i,:)=((jacobian(PHIs(i,:),Bod.L))*F(i,:)'); end; %  tau=JT*F

end
%  




































































% global PHIs 
% if ~exist('showIt','var'); showIt = 0; end %default
% 
% % Define Range and Mean for X and Y
% data = load('AttractorFieldData.txt');
% X = data(:,1);
% Y = data(:,2);
% [x,y] = meshgrid(X,Y);
% %X = [0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.47	0.22	0.22	0.47];
% %Y = [0.1	0.1	0.1	0.1	0.1	0.075	0.075	0.075	0.075	0.075	0.05	0.05	0.05	0.05	0.05	0.025	0.025	0.025	0.025	0.025	0	0	0	0	0	-0.025	-0.025	-0.025	-0.025	-0.025	-0.05	-0.05	-0.05	-0.05	-0.05	-0.075	-0.075	-0.075	-0.075	-0.075	-0.1	-0.1	-0.1	-0.1	-0.1	-0.125	-0.125	-0.125	-0.125	-0.125	-0.15	-0.15	-0.15	-0.15	-0.15	-0.175	-0.175	-0.175	-0.175	-0.175	-0.2	-0.2	-0.2	-0.2	-0.2	-0.225	-0.225	-0.225	-0.225	-0.225	-0.25	-0.25	-0.25	-0.25	-0.25	-0.07	-0.08	-0.07	-0.08];
% scatter(X,Y);
% mu = [mean(X) mean(Y)];
% size(X,1);
% % Create matrix for X and Y
% 
% XY = [x,y];
% 
% % Calculate Covariance of x,y matrix to determine Sigma
% Sigma = cov(X,Y);
% 
% % Calculate Multivariate-Normal Probability Density Function
% % F = mvnpdf([x(:) y(:)], mu, Sigma);
% % F = reshape(F,length(X),length(Y));
% 
% % [Dx Dy] = gradient(F,.1,.2);
% % Dx = transpose(Dx(1,:));
% % Dy = transpose(Dy(1,:));
% % DXY = [Dx Dy];
% 
% % % Plot MVN and Quiver Plot
% % figure
% % contour(x,y,F)
% % hold on
% % quiver(x,y,Dx,Dy)
% 

%F = [Fx Fy]


% m = mean(r);
%plot(r1,F)
% % Fx(1) = 0;
% for i = 2:length(r)
%      Fx(i,:) = r(i)-r(i-1);
% end
% Fy(1) = 0;
% for i = 2:length(r)
%     Fy(i,:) = F(i)-F(i-1);
% end
% 
% G = [Fx Fy];

           