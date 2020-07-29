%% Attractor Field

function [TAUs,PHIs,Pos]=AttractorField(Bod);  
global PHIs XX DXX
if ~exist('showIt','var'); showIt = 0; end %default

%% Method 1
% %Define Range and Mean for X and Y
%     X = .25:.01:.45;
%     Y = -1:.01:1;
%     X = transpose(X);
%     Y = transpose(Y);
    
X = [0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.25	0.3	0.35	0.4	0.45	0.47	0.22	0.22	0.47];
Y = [0.1	0.1	0.1	0.1	0.1	0.075	0.075	0.075	0.075	0.075	0.05	0.05	0.05	0.05	0.05	0.025	0.025	0.025	0.025	0.025	0	0	0	0	0	-0.025	-0.025	-0.025	-0.025	-0.025	-0.05	-0.05	-0.05	-0.05	-0.05	-0.075	-0.075	-0.075	-0.075	-0.075	-0.1	-0.1	-0.1	-0.1	-0.1	-0.125	-0.125	-0.125	-0.125	-0.125	-0.15	-0.15	-0.15	-0.15	-0.15	-0.175	-0.175	-0.175	-0.175	-0.175	-0.2	-0.2	-0.2	-0.2	-0.2	-0.225	-0.225	-0.225	-0.225	-0.225	-0.25	-0.25	-0.25	-0.25	-0.25	-0.07	-0.08	-0.07	-0.08];
X = transpose(X);
Y = transpose(Y);
   mu = [mean(X) mean(Y)];
%  % Create matrix for X and Y
  [x,y] = meshgrid(X,Y);
%  
%  
  XX = [x,y];
% %Calculate covariance of x,y matrix to determine Sigma
  Sigma = cov(x,y);
% % probability density function for MVN Distribution
  F = mvnpdf([x(:) y(:)], mu, Sigma); 
  F = reshape(F,length(X),length(Y));
%surf(x,y,F)

[Dx Dy] = gradient(F,.1,.2);

% x = [transpose(x(1,:));transpose(x(2,:));transpose(x(3,:));transpose(x(4,:));transpose(x(5,:));transpose(x(6,:));transpose(x(7,:));transpose(x(8,:));transpose(x(9,:));transpose(x(10,:));transpose(x(11,:))]
% y = [transpose(y(1,:));transpose(y(2,:));transpose(y(3,:));transpose(y(4,:));transpose(y(5,:));transpose(y(6,:));transpose(y(7,:));transpose(y(8,:));transpose(y(9,:));transpose(y(10,:));transpose(y(11,:))]
% Dx = [transpose(Dx(1,:));transpose(Dx(2,:));transpose(Dx(3,:));transpose(Dx(4,:));transpose(Dx(5,:));transpose(Dx(6,:));transpose(Dx(7,:));transpose(Dx(8,:));transpose(Dx(9,:));transpose(Dx(10,:));transpose(Dx(11,:))]
% Dy = [transpose(Dy(1,:));transpose(Dy(2,:));transpose(Dy(3,:));transpose(Dy(4,:));transpose(Dy(5,:));transpose(Dy(6,:));transpose(Dy(7,:));transpose(Dy(8,:));transpose(Dy(9,:));transpose(Dy(10,:));transpose(Dy(11,:))]
XX = [x y];
DXX = [Dx Dy];
%quiver(x,y,Dx,Dy) 
%hold on
plot(x,y,'.','color',.6*[1 1 1])% plot positions grey
hold on
%quiver(x,y,Dx,Dy);

%% Method 2
% for i = 1:size(PHIs,1)
%     TAUs(i,:) = sqrt(Dx(i).^2+Dy(i).^2);
%     equivWristForce = (inv (jacobian(PHIs(i,:),Bod.L)') * TAUs(i,:)')';
%     if showIt
%         simpleArrow(Pos.wr(i,:),Pos.wr(i,:)+equivWristForce,'y',1); 
%     end

%% Method that's giving me errors
PHIs=inverseKin(XX,Bod.L); % 
Pos=forwardKin(PHIs,Bod);   % positions assoc w/ these angle combinations
for i=1:size(XX,1), TAUs(i,:)=((jacobian(PHIs(i,:),Bod.L)')*transpose(DXX(i,:))); end; % tau=JT*F


end

