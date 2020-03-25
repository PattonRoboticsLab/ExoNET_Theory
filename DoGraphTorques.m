% ***********************************************************************
% Plot the ExoNET torques superimposed on the desired torques
% ***********************************************************************

subplot(2,1,1)
plot(time,TAUsDESIRED(:,1),'r','LineWidth',2)
hold on
plot(time,TAUs(:,1),'b','LineWidth',2)
xlabel('time [s]'); ylabel('Hip Moment [Nm]'); %legend('Desired','ExoNET');
plot(time(1),TAUsDESIRED(1,1),'.k'); plot(time(70),TAUsDESIRED(70,1),'.k'); % TOR
text(time(1),TAUsDESIRED(1,1)+3,'TOR'); text(time(70),TAUsDESIRED(70,1)-5,'TOR');
plot(time(28),TAUsDESIRED(28,1),'.k'); plot(time(97),TAUsDESIRED(97,1),'.k'); % HCR
text(time(28)+0.01,TAUsDESIRED(28,1)+2,'HCR'); text(time(97),TAUsDESIRED(97,1)+7,'HCR');

subplot(2,1,2)
plot(time,TAUsDESIRED(:,2),'r','LineWidth',2)
hold on
plot(time,TAUs(:,2),'b','LineWidth',2)
xlabel('time [s]'); ylabel('Knee Moment [Nm]'); %legend('Desired','ExoNET');
plot(time(1),TAUsDESIRED(1,2),'.k'); plot(time(70),TAUsDESIRED(70,2),'.k'); % TOR
text(time(1),TAUsDESIRED(1,2)-5,'TOR'); text(time(70),TAUsDESIRED(70,2)+3,'TOR');
plot(time(28),TAUsDESIRED(28,2),'.k'); plot(time(97),TAUsDESIRED(97,2),'.k'); % HCR
text(time(28)+0.01,TAUsDESIRED(28,2),'HCR'); text(time(97),TAUsDESIRED(97,2)-5,'HCR');

