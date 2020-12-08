% plot1DDistribution:   plot 1-D distibution
% An alterantive to box plot - dots scattered to look like a histogram 
% SYNTAX:   plot1DDistribution(y,c,xLims)
%           where   y       is the data, in no particular order
%                   c       (optional) is the color sring (example: 'b')
%                   xLims   (optional) is the limits to x [lo,hi]
% examples: 
%   clf; N=50; for i=1:2; plot1DDistribution(rand(1,1)+randn(1,N),'b',i+[-.1 .1]); end;
%  OR:
%   x=[rand(500,1); 3+randn(1000,1); -1+randn(2000,1)]; % create random values with bizzare distribution
%   plot1DDistribution(x)
% MODIFIED: 2013-Nov-21 created by patton
% === BEGIN: ===

function plot1DDistribution(y,c,xLims)

%% setup
fcnName='plot1DDistribution';
Gr=.7*ones(3,1); % RGB color grey
fprintf(['~ ' fcnName ' ~ '])
if ~exist('y','var');
  fprintf('\n  ! no input !');
  fprintf('\n     % SYNTAX:   plot1DDistribution(y,colorString,xLims)');
  fprintf('\n   Creating random example data ...');
  y=[randn(50,1); 3+randn(50,1)] 
end
[nR,nC]=size(y); if nR<nC, y=y'; end; N=length(y);
if size(y,2)>1, disp(' input y must be a vector. '); return; end
if ~exist('c','var'); c='k'; end % color black if not given

% create x variables
if ~exist('xLims','var'), 
  x=(((1:length(y))-1).*.005)';  % staggered x
  xLims=[min(x) max(x)]; 
else
  x=(xLims(1)+((0:1/(N-1):1)).*range(xLims))'; % staggered x
end
xRange=abs(xLims(2)-xLims(1));
xCenter=mean(xLims); 
x1=[(x-xRange/40) (x+xRange/40)]; % raster line limits
%plot(x,y,['g.'],'markersize', 1); hold on; % plot raw 
%y=y+rand(size(y))*range(y)/500;   % slight noisify y for visibility

%% stagger datapoints in x based their density at y
[~,I]=sort(rand(size(y))); y=y(I); % randomly-reorder y

% create a distribution and present it shaded
% yPD=fitdist(y,'normal'); % fit normal probability distribution
%yPD = fitdist(y,'Kernel','Kernel','epanechnikov'); % multi w/epanechnikov
%smoothing - see https://en.wikipedia.org/wiki/Kernel_density_estimation
% https://en.wikipedia.org/wiki/Kernel_(statistics)#Kernel_functions_in_common_use
yPD = fitdist(y,'Kernel','Kernel','Epanechnikov'); % multi w/epanechnikov smoothing
X=(min(y):(max(y)-min(y))/30:max(y))';
FofX=pdf(yPD,X);
xx=[xCenter+FofX/2; xCenter-flipud(FofX)/2]; yy=[X; flipud(X)]; 
hh=patch(xx,yy,'y');  
myYellow=[1 1 .9]; % bacground shading color
set(hh,'FaceColor',myYellow,'FaceAlpha',.5,'EdgeColor','none')

hold on

% reorder the x
M=round(N/2); % index of the midpoint 
maxY=max(y); minY=min(y); rangeY=maxY-minY;
for j=1:ceil(N/10),    % repeat this process below
  for i=1:2:N%round(rand(1)*10):N, % each point
    NormDistFromMax=(maxY-y(i))/rangeY;
    NormDistFromMin=(y(i)-minY)/rangeY;
    %   NormDistFromEither=min([NormDistFromMin,NormDistFromMax]);
    %   p=round(M+(NormDistFromEither*N/2));
    %   yT=y(i); y(i)=[]; y=[y(1:p); yT; y(p+1:N-1)]; % copy, snip, put in mid
    PorM=1;round(rand(1))*2-1; % random either + or - 1
    if NormDistFromMax < .5 % top half
      p=M+PorM*round(NormDistFromMax*N/2);
      yT=y(i); y(i)=[]; y=[y(1:p); yT; y(p+1:N-1)]; % copy, snip, put in mid
    elseif NormDistFromMin < .5; % bottom half
      p=M-PorM*round(NormDistFromMin*N/2);
      yT=y(i); y(i)=[]; y=[y(1:p); yT; y(p+1:N-1)]; % copy, snip, put in mid
    end
  end
end
% y=[	(y(1:2:round(N/2))); y(N:-2:round(N/2)+1)]; % mix
% y=[	(y(1:2:N)); y(N:-2:round(N/2)+1)]; % mix

% mix left and right
for i=1:2:floor(N/2), % every other  
  y=swapRows(y,i,N-i);
end

% mix outside
for j=1:1
  for i=1:2:floor(N/4), % bring in half of the outside fourths
    p=M-i; yT=y(i); y(i)=[]; y=[y(1:p); yT; y(p+1:N-1)]; % move in mid
    p=M+i; yT=y(N-i); y(N-i)=[]; y=[y(1:p); yT; y(p+1:N-1)]; % move in mid
  end
end

%% plot
% size(x), size(y)
%plot(x,y,[c 'o'],'linewidth',.0001, 'markersize', 2); hold on

% raster Lines
for i=1:N; 
   %plot(x1(i,:),y(i)*[1 1],'b-','linewidth',.001); hold on
   XX=xCenter+(rand(1)-.5)*pdf(yPD,y(i)); YY=y(i);plot(XX,YY,'o'); hold on
   %plot(xCenter+(rand(1)-.5)*pdf(yPD,y(i)),y(i),'b.','linewidth',.001); hold on
   %plot(x1(i,:),y(i)*[1 1],[c '-'],'linewidth',.001); hold on
end



% confidence & mean
wing=confidence(y,.95); 
plot(mean(x)*[1 1],mean(y)+[wing -wing],c, 'linewidth',6, 'Color',Gr); 
hold on
plot(mean(x)*[1 1],mean(y)+[std(y) -std(y)],c, 'linewidth',3, 'Color',Gr); 
plot(mean(x)*[1 1],mean(y)+[std(y) -std(y)],'w', 'linewidth',1); 
plot(mean(x),mean(y),['k.'],'markersize', 8); 

text(mean(x),mean(y),'     Mean','fontSize',6, 'Color',Gr);
text(mean(x),mean(y)+wing,'     95% confidence','fontSize',6, 'Color',Gr);
text(mean(x),mean(y)+std(y),'   +1 Stan. Dev.','fontSize',6, 'Color',Gr);

% bound window if no bounds given
if ~exist('xLims','var');
    ax=axis; % get limits
    %axis([min(x),max(x),ax(3),ax(4)])
    %axis([-.3,.3,0,20])
end
%set(gca,'xTick',[],'box','off')

%% wrapup
drawnow
fprintf(['\n~ END '  fcnName ' ~ \n'])

end


function d=swapRows(d,i,j,verbose)

%____ SETUP ____
if ~exist('verbose'), verbose=0; end
if verbose, fprintf('~ %s ~ ', prog_name); end 

q=d(j,:); 
d(j,:)=d(i,:); 
d(i,:)=q;

if verbose, fprintf('~ END %s ~ ', prog_name); end 

end
