%% Attractor Field


function [TAUs,PHIs,Pos]=AttractorField(Bod);                     % set torques2cancelGrav
%clc
%clear all
%close all
%Define Range and Mean for X and Y
X = 0:.2:2;
Y = 0:.4:4;
mu = [mean(X) mean(Y)];

% Create matrix for X and Y
[x,y] = meshgrid(X,Y);
XX = [x,y];

%Calculate covariance of x,y matrix to determine Sigma
Sigma = cov(x,y);


% for i = 1:length(X),
%     for j = 1:length(Y),
%         for z = 1:length(X)
%             XX(z,:) = [X(i) Y(j)];
%         end
%       end
% end

% probability density function for MVN Distribution

F = mvnpdf([x(:) y(:)], mu, Sigma); 
F = reshape(F,length(X),length(Y));
% surf(x,y,F)

[Dx Dy] = gradient(F,.1,.2);
figure
contour(x,y,F)
hold on
quiver(x,y,Dx,Dy)
hold off

x = transpose(x(1,:));
y = y(:,1);
X = [x y];
F = [Dx(:,1) Dy(:,1)];


%plot(X(:,1),X(:,2),'.','color',.6*[1 1 1]); % plot positions grey

% PHIs=inverseKin(X,Bod.L); % 
% Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations
% 
% for i=1:size(X), TAUs(i,:)=((jacobian(PHIs(i,:),Bod.L)')*F(i,:)'); end; % tau=JT*F
% 
% 
% 
