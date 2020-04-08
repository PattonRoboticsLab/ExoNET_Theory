% ***********************************************************************
% Plot the ExoNET torques superimposed on the desired torques
% ***********************************************************************

figure
subplot(2,1,1)
p1 = plot(time,TAUsDESIRED(:,1),'r','LineWidth',2);
hold on
p2 = plot(time,TAUs(:,1),'b','LineWidth',2);
xlabel('time [s]'); ylabel('Hip Moment [Nm]');
p3 = plot(time(1),TAUsDESIRED(1,1),'.k'); p4 = plot(time(70),TAUsDESIRED(70,1),'.k'); % TOR
text(time(1),TAUsDESIRED(1,1)+3,'TOR'); text(time(70),TAUsDESIRED(70,1)-5,'TOR');
p5 = plot(time(28),TAUsDESIRED(28,1),'.k'); p6 = plot(time(97),TAUsDESIRED(97,1),'.k'); % HCR
text(time(28)+0.01,TAUsDESIRED(28,1)+2,'HCR'); text(time(97),TAUsDESIRED(97,1)+7,'HCR');
legend([p1 p2],'Desired','ExoNET');

subplot(2,1,2)
p1 = plot(time,TAUsDESIRED(:,2),'r','LineWidth',2);
hold on
p2 = plot(time,TAUs(:,2),'b','LineWidth',2);
xlabel('time [s]'); ylabel('Knee Moment [Nm]');
p3 = plot(time(1),TAUsDESIRED(1,2),'.k'); p4 = plot(time(70),TAUsDESIRED(70,2),'.k'); % TOR
text(time(1),TAUsDESIRED(1,2)-5,'TOR'); text(time(70),TAUsDESIRED(70,2)+3,'TOR');
p5 = plot(time(28),TAUsDESIRED(28,2),'.k'); p6 = plot(time(97),TAUsDESIRED(97,2),'.k'); % HCR
text(time(28)+0.01,TAUsDESIRED(28,2),'HCR'); text(time(97),TAUsDESIRED(97,2)-6,'HCR');
legend([p1 p2],'Desired','ExoNET');

