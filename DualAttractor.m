%% Dual Attractor Field
function [TAUs_1,PHIs,Pos]=DualAttractor(Bod);  

global F TAUs_1

%% Initialize Range of Attractor 1
%x min,max, range
min_x = .1;
max_x = .3;
x1range = min_x:.025:max_x;

%y min,max, range
min_y =-0.3;
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
max_x = .3;
x2range = min_x:.025:max_x;

%y min,max, range
min_y =.1;
max_y = 0.25;
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
for i = 1:size(x1,1)
    r1(i,:) = center1-x1(i,:);
    R1(i,:) = 20*norm(r1(i,:));
end
for i = 1:size(x2,1)
    r2(i,:) = center2-x2(i,:);
    R2(i,:) = 20*norm(r2(i,:));
end


meanR1 = (max(R1)-min(R1))/2;
meanR2 = (max(R2)-min(R2))/2;

Fx1 = zeros(size(r1));
Fx2 = zeros(size(r2));
for i = 1:size(r1,1)
    F1x1(i) = exp(-1*(R1(i)-meanR1).^2/(2));
    Fx1(i,:) = transpose(F1x1(i).*r1(i,:)');
end

for i = 1:size(r2,1)
F1x2(i) = exp(-1*(R2(i)-meanR2).^2/(2));
    Fx2(i,:) = transpose(F1x2(i).*r2(i,:)');
end

    


%% Initialize Force Matrix (F1(i,1) = Fx and F1(i,2) = Fy)
F = [Fx1;Fx2];



%% Connect to Main

plot(x(:,1),x(:,2),'.','color',.8*[1 1 1]); % plot positions grey
PHIs=inverseKin(x,Bod.L); % 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations

TAUs_1 = zeros(size(x));
for i=1:size(x,1), TAUs_1(i,:)=((jacobian(PHIs(i,:),Bod.L))*F(i,:)'); end; %  tau=JT*F

end
