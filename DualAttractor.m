%% Dual Attractor Field
function [TAUs_1,PHIs,Pos]=DualAttractor(Bod);  

global F TAUs_1

%% Initialize Range of Attractor 1
%x min,max, range
min_x = .1;
max_x = .2;
x1range = min_x:.025:max_x;

%y min,max, range
min_y =-0.1;
max_y = 0;
y1range = min_y:.025:max_y;

% All combinations of X and Y in x matrix
[X Y ] = meshgrid(x1range,y1range);
c=cat(2,X',Y');
newmatrix = reshape(c,[],2);
x1 = newmatrix(:,1:2);

%Center of Attractor
center1 = [mean(x1range) mean(y1range)];


%% Initialize Range of Attractor 2
%x min,max, range
min_x = .1;
max_x = .2;
x2range = min_x:.025:max_x;

%y min,max, range
min_y =.1;
max_y = 0.2;
y2range = min_y:.025:max_y;

% All combinations of X and Y in x matrix
[X2 Y2 ] = meshgrid(x2range,y2range);
c=cat(2,X2',Y2');
newmatrix2 = reshape(c,[],2);
x2 = newmatrix2(:,1:2);

%Center of Attractor 2
center2 = [mean(x2range) mean(y2range)];

%% X Matrix of Positions for both Attractors
x = [x1;x2];

%% Define Mean and Sigma of Gaussian Distribution
mu = [0.05 0.05]; % max force at this distance from center
Sigma = [1 0; 0 1]; % variance (sigma) for the Gaussian

%% Calculate Multivariate Gaussian
Fx1 = mvnpdf([x1(:,1) x1(:,2)],mu,Sigma);
Fx2 = mvnpdf([x2(:,1) x2(:,2)],mu,Sigma);
%% Initialize Force Matrix (F1(i,1) = Fx and F1(i,2) = Fy)
F1x1 = zeros(size(x1));
F1x2 = zeros(size(x2));
for i = 1:size(x1,1)
    r(i,:) = center1-x1(i,:);%x(i,:)-x01;
    F1x1(i,:) = Fx1(i).*r(i,:);       
end
for i = 1:size(x2,1)
    r(i,:) = center2-x2(i,:);
    F1x2(i,:) = Fx2(i).*r(i,:);      
end

F = [F1x1;F1x2];



%% Connect to Main

plot(x(:,1),x(:,2),'.','color',.8*[1 1 1]); % plot positions grey
PHIs=inverseKin(x,Bod.L); % 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations

TAUs_1 = zeros(size(x));
for i=1:size(x,1), TAUs_1(i,:)=((jacobian(PHIs(i,:),Bod.L))*F(i,:)'); end; %  tau=JT*F

end
