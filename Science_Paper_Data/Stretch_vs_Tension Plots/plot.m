clc
clear all
close all
% Load saved figures
c=hgload('gravcomp_stretch.fig');
k=hgload('singleattractor_stretch.fig');
m = hgload('dualattractor_stretch.fig');
one = hgload('LP_stretch.fig');
two = hgload('EA_stretch.fig');
% Prepare subplots
figure

h(1)=subplot(2,3,1);
h(2)=subplot(2,3,2);
h(3) = subplot(2,3,3);
h(4)=subplot(2,3,4);
h(5)=subplot(2,3,5);



% Paste figures on the subplots

copyobj(allchild(get(c,'CurrentAxes')),h(1));
copyobj(allchild(get(k,'CurrentAxes')),h(2));
copyobj(allchild(get(m,'CurrentAxes')),h(3));
copyobj(allchild(get(one,'CurrentAxes')),h(4));
copyobj(allchild(get(two,'CurrentAxes')),h(5));

%copyobj(allchild(get(m,'CurrentAxes')),h(4));

% Add legends

% l(1)=legend(h(1),'LegendForFirstFigure')
% l(2)=legend(h(2),'LegendForSecondFigure')
% l(3)=legend(h(3),'LegendForFirstFigure')
% l(4)=legend(h(4),'LegendForSecondFigure')