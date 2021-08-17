% Load saved figures
c=hgload('1e2j_gc.fig');
k=hgload('1e_gc.fig');
m = hgload('5e_gc.fig');
one = hgload('1e_2j.fig');
two = hgload('1e_EA.fig');
three = hgload('5e_gc.fig');
% Prepare subplots
figure

h(1)=subplot(2,3,1);
axis off
h(2)=subplot(2,3,2);
axis off
h(3) = subplot(2,3,3);
axis off
h(4)=subplot(2,3,4);
axis off
h(5)=subplot(2,3,5);
axis off
h(6)=subplot(2,3,6);
axis off


% Paste figures on the subplots

copyobj(allchild(get(c,'CurrentAxes')),h(1));
copyobj(allchild(get(k,'CurrentAxes')),h(2));
copyobj(allchild(get(m,'CurrentAxes')),h(3));
copyobj(allchild(get(one,'CurrentAxes')),h(4));
copyobj(allchild(get(two,'CurrentAxes')),h(5));
copyobj(allchild(get(three,'CurrentAxes')),h(6));

%copyobj(allchild(get(m,'CurrentAxes')),h(4));

% Add legends

% l(1)=legend(h(1),'LegendForFirstFigure')
% l(2)=legend(h(2),'LegendForSecondFigure')
% l(3)=legend(h(3),'LegendForFirstFigure')
% l(4)=legend(h(4),'LegendForSecondFigure')