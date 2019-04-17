%% Single Attractor Field
function [TAUs_1,PHIs,Pos]=SingleAttractor(Bod);  

global TAUs_1 F

%% Initialize Range of Attractor

%x min,max, range
min_x = .15;
max_x = .3;
xrange = min_x:.025:max_x;

%y min,max, range
min_y =-.1;
max_y = 0;
yrange = min_y:.025:max_y;

% All combinations of X and Y in x matrix
[X Y ] = meshgrid(xrange,yrange);
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
%F1 = mvnpdf([x(:,1) x(:,2)],center,Sigma)
%%

for i = 1:size(x,1)
    r(i,:) = center-x(i,:); 
    r1(i,:) =10* norm(r(i,:));
end

meanr = (max(r1)-min(r1))/2;

F = zeros(size(r));
for i = 1:size(r,1)
    %F1(i,:) = exp(-1*(r1(i)-meanr).^2/(2));
    %F1(i,:) = exp(r1(i))/(exp(r1(i))+1);
    F1(i,:) = r1(i)/sqrt(1+r1(i).^2);
    F(i,:) =transpose( F1(i).*r(i,:)');
         
      %plot(r1(i),F1(i),'bo');
      %hold on
end

plot(x(:,1),x(:,2),'.','color',.8*[1 1 1]); % plot positions grey
PHIs=inverseKin(x,Bod.L); % 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations

TAUs_1 = zeros(size(x));
for i=1:size(x,1), TAUs_1(i,:)=((jacobian(PHIs(i,:),Bod.L))*F(i,:)'); end; %  tau=JT*F

end
