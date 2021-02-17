%% Single Attractor Field
function [TAUs,PHIs,Pos]=SingleAttractor(Bod);  
global ProjectName
ProjectName='SingleAttractor';
title(ProjectName);

%% Initialize Range of Attractor

%x min,max, range
min_x = 0.15;
max_x = .45;
xrange = min_x:.05:max_x;

%y min,max, range
min_y =-.25;
max_y = .25;
yrange = min_y:.05:max_y;

% All combinations of X and Y in x matrix
[X, Y] = meshgrid(xrange,yrange);
c=cat(2,X',Y');
newmatrix = reshape(c,[],2);
x = newmatrix(:,1:2);

%Center of Attractor
center = [mean(xrange) mean(yrange)];
Sigma = [1 0;0 1];

%% Define Mean and Sigma of Gaussian Distribution
mu = [0 0]; % max force at this distance from center
Sigma = [.05 0; 0 .05]; % variance (sigma) for the Gaussian

%% Calculate Multivariate Gaussian Function
%F1 = mvnpdf([x(:,1) x(:,2)],mu,Sigma)
%%

for i = 1:size(x,1)
    r(i,:) = center-x(i,:); 
    r1(i) =(2/3)*norm(r(i,:));
end

s = std(r1);
meanr = (mean(r1));

F = zeros(size(r));
meanr1 = mean(r1);
for i = 1:size(r,1)
  %% Calculating Force Using Different Function
  % Gaussian
  % F1(i,:) = (1/(sqrt(2*pi*s.^2)))*exp(-(r1(i)-meanr).^2/(2*s.^2));
  
  % Sigmoid
  F1(i,:) = 1/(1+exp((-(r1(i)-meanr)/(s^2*pi^2/3))));
  if F1(i) > 0.0000001
    F1(i) = F1(i)+1;
  else
    F1(i) = 0;
  end
  
  F(i,:) =transpose( F1(i).*(r(i,:)./r1(i))');    % Force Calculation
end

plot(x(:,1),x(:,2),'.','color',.8*[1 1 1]) % plot positions grey
PHIs=inverseKin(x,Bod.L);    % 
Pos=forwardKin(PHIs,Bod);    % positions assoc w/ these angle combinations

TAUs = zeros(size(x));
for i=1:size(x,1)
  TAUs(i,:)=((jacobian(PHIs(i,:),Bod.L)')*F(i,:)');  %  tau=JT*F
end
TAUs = TAUs * 2;
end