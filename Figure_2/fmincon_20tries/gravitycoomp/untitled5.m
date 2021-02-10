close all
seven =hgload('1e2j_dualattract.fig');
eight = hgload('1e_dualattract.fig');
nine = hgload('5e_dualattract.fig');
figure

h(1)=subplot(1,1,1);
% h(2)=subplot(1,4,2);
%h(3) = subplot(1,4,3);
%h(4) = subplot(1,4,4);
%copyobj(allchild(get(seven, 'CurrentAxes')),h(1));
%%copyobj(allchild(get(eight,'CurrentAxes')),h(2));
copyobj(allchild(get(nine,'CurrentAxes')),h(1));