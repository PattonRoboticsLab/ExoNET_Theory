%% Single Attractor Field
function [TAUs_1,PHIs,Pos]=SingleAttractor(Bod);  

global F TAUs_1

%% Initialize Range of Attractor

%x min,max, range
min_x = .1;
max_x = .5;
xrange = min_x:.025:max_x;

%y min,max, range
min_y =-.1;
max_y = .3;
yrange = min_y:.05:max_y;

% All combinations of X and Y in x matrix
[X Y ] = meshgrid(xrange,yrange);
c=cat(2,X',Y');
newmatrix = reshape(c,[],2);
x = newmatrix(:,1:2);

%Center of Attractor
center = [mean(xrange) mean(yrange)];
Sigma = [1 0;0 1];

%% Define Mean and Sigma of Gaussian Distribution
%mu = [0.05 0.05]; % max force at this distance from center
%Sigma = [1 0; 0 1]; % variance (sigma) for the Gaussian

%% Calculate Multivariate Gaussian Function
F1 = mvnpdf([x(:,1) x(:,2)],center,Sigma);



%% Initialize Force Matrix (F1(i,1) = Fx and F1(i,2) = Fy)
F = zeros(size(x));
for i = 1:size(x,1)
    r(i,:) = center-x(i,:);
    F(i,:) = F1(i).*r(i,:);
    r1(i) = norm(r(i,:));
    
    
    plot(-r1(i),F1(i),'bo',r1(i),F1(i),'bo')
    hold on
    
    
        
end
%hold off
%% Connect to Main

%plot(x(:,1),x(:,2),'.','color',.8*[1 1 1]); % plot positions grey
PHIs=inverseKin(x,Bod.L); % 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations

TAUs_1 = zeros(size(x));
for i=1:size(x,1), TAUs_1(i,:)=((jacobian(PHIs(i,:),Bod.L))*F(i,:)'); end; %  tau=JT*F

end
