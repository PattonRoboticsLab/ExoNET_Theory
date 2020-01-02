% x0=.3; 
% y0=.05; 
% sigma=1; 
% x=.1:.07:.3;
% y=-.2:.07:.1;
%  
% X=NaN*zeros(i*j,2);  % init
% count=0;
% for i=1:length(x), 
%   for j=1:length(y), 
%     count=count+1; 
%     X(count,:)=[x(i) y(j)];
%   end; 
% end;  
% X;
%  
% plot3(X(i,1),X(i,2),x0+exp(-X(:,1).^2/sigma^2),'.'),
clc
close all
clear all
x0 = 0.3;
mu = [0 0];
Sigma = [.25 .3; .3 1];
x1 = -3:.2:3; x2 = -3:.2:3;
X=NaN*zeros(i*j,2);
[X1,X2] = meshgrid(x1,x2);
count = 0;
for i = 1:length(x1),
    for j = 1:length(x2),
        count = count +1;
        X(count,:) = [x1(i) x2(j)];
    end
end
X
plot3(X(i,1),X(i,2),x0+exp(-X(:,1).^2./Sigma(1).^2),'.'),
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(x2),length(x1));
surf(x1,x2,F);
caxis([min(F(:))-.5*range(F(:)),max(F(:))]);
axis([-3 3 -3 3 0 .4])
xlabel('x1'); ylabel('x2'); zlabel('Probability Density');


syms x y
f = -(sin(x) + sin(y))^2;
g = gradient(f, [x, y])

% [X, Y] = meshgrid(-1:.1:1,-1:.1:1);
% G1 = subs(g(1), [x y], {X,Y});
% G2 = subs(g(2), [x y], {X,Y});
% quiver(X, Y, G1, G2)