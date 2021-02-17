function plot2DDistribution(x,y,c)

Gr = 0.7*ones(3,1); % RGB color grey

[nRx,nCx] = size(x);
if nRx<nCx; x = x'; end; M = length(x);

[nRy,nCy] = size(y);
if nRy<nCy; y = y'; end; N = length(y);

for i = 1:N
    XX = x(i);
    YY = y(i);
    plot(XX,YY,'o','LineWidth',1,'MarkerEdgeColor','w','MarkerFaceColor',c);
    hold on
end

% hh = patch(x,y,'y');
% myYellow = [1 1 0.9]; % background shading color
% set(hh,'FaceColor',myYellow,'FaceAlpha',0.5,'EdgeColor','none')
% hold on

%plot([-5 5],[0 0],'k','LineWidth',3)
%plot([0 0],[-5 5],'k','LineWidth',3)

line([0 0], ylim,'Color','k','LineWidth',1.5)
line(xlim, [0 0],'Color','k','LineWidth',1.5);
% Confidence & Mean x-axis
% Xwing = confidence(x,0.95);
% plot(mean(x)+[Xwing -Xwing],mean(y)*[1 1],'k','linewidth',6,'Color',Gr);
% hold on
% plot(mean(x)+[std(x) -std(x)],mean(y)*[1 1],'k','linewidth',3,'Color',Gr);
% plot(mean(x)+[std(x) -std(x)],mean(y)*[1 1],'w','linewidth',1);
% plot(mean(x),mean(y),['k.'],'markersize',8);

% text(mean(x),mean(y)+0.8,'     Mean','fontSize',8,'Color','k');
% text(mean(x)+Xwing-1,mean(y)-0.6,'     95% Confidence','fontSize',8,'Color','k');
% text(mean(x)+std(x),mean(y),'   +1 Stan. Dev.','fontSize',8,'Color','k');



% Confidence & Mean y-axis
% Ywing = confidence(y,0.95);
% plot(mean(x)*[1 1],mean(y)+[Ywing -Ywing],'k','linewidth',6,'Color',Gr);
% hold on
% plot(mean(x)*[1 1],mean(y)+[std(y) -std(y)],'k','linewidth',3,'Color',Gr);
% plot(mean(x)*[1 1],mean(y)+[std(y) -std(y)],'w','linewidth',1);
% plot(mean(x),mean(y),['k.'],'markersize',8);

% text(mean(x),mean(y)+Ywing,'     95% Confidence','fontSize',8,'Color','k');
% text(mean(x),mean(y)+std(y),'   +1 Stan. Dev.','fontSize',8,'Color','k');


box off
% axis equal
axis image
% axis([-40 25 -40 25])

% title('Error Distribution')
xlabel('Shoulder Torque Error [Nm]')
ylabel('Elbow Torque Error [Nm]')

end