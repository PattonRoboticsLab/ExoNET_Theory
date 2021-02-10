close all
% Load saved figures
one=hgload('1e2j_gc.fig');
two=hgload('1e_gc.fig');
three=hgload('5e_gc.fig');
four = hgload('1e2j_singleattract.fig');
five = hgload('1e_singleattract.fig');
six = hgload('5e_singleattract.fig');
seven = hgload('1e2j_LP.fig');
eight =hgload('1e_LP.fig');
nine =hgload('5e_LP.fig');
ten =hgload('1e2j_dualattract.fig');
eleven = hgload('1e_dualattract.fig');
twelve = hgload('5e_dualattract.fig');
thirteen =hgload('1e2j_EA.fig');
fourteen = hgload('1e_EA.fig');
fifteen  = hgload('5e_EA.fig');



% Prepare subplots
% figure
% 
% h(1)=subplot(5,3,1);
% h(2)=subplot(5,3,2);
% h(3) = subplot(5,3,3);
% h(4)=subplot(5,3,4);
% h(5)=subplot(5,3,5);
% h(6)=subplot(5,3,6);
% h(7)=subplot(5,3,7);
% h(8)=subplot(5,3,8);
% h(9) = subplot(5,3,9);
% h(10)=subplot(5,3,10);
% h(11)=subplot(5,3,11);
% h(12)=subplot(5,3,12);
% h(13)=subplot(5,3,13);
% h(14)=subplot(5,3,14);
% h(15)=subplot(5,3,15);
% 
% 
% % Paste figures on the subplots
% copyobj(allchild(get(one, 'CurrentAxes')),h(1));
% copyobj(allchild(get(two,'CurrentAxes')),h(2));
% copyobj(allchild(get(three,'CurrentAxes')),h(3));
% copyobj(allchild(get(four,'CurrentAxes')),h(4));
% copyobj(allchild(get(five,'CurrentAxes')),h(5));
% copyobj(allchild(get(six,'CurrentAxes')),h(6));
% copyobj(allchild(get(seven,'CurrentAxes')),h(7));
% copyobj(allchild(get(eight,'CurrentAxes')),h(8));
% copyobj(allchild(get(nine,'CurrentAxes')),h(9));
% copyobj(allchild(get(ten,'CurrentAxes')),h(10));
% copyobj(allchild(get(eleven,'CurrentAxes')),h(11));
% copyobj(allchild(get(twelve,'CurrentAxes')),h(12));
% copyobj(allchild(get(thirteen,'CurrentAxes')),h(13));
% copyobj(allchild(get(fourteen,'CurrentAxes')),h(14));
% copyobj(allchild(get(fifteen,'CurrentAxes')),h(15));


%copyobj(allchild(get(m,'CurrentAxes')),h(4));

% Add legends

% l(1)=legend(h(1),'LegendForFirstFigure')
% l(2)=legend(h(2),'LegendForSecondFigure')
% l(3)=legend(h(3),'LegendForFirstFigure')
% l(4)=legend(h(4),'LegendForSecondFigure')