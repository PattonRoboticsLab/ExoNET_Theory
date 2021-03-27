% ***********************************************************************
% Plot the ExoNET torques superimposed on the desired torques
% ***********************************************************************

function showGraphTorquesLeg(percentageGaitCycle,TAUsDESIRED,TAUs)

figure
subplot(2,1,1)
p1 = plot(percentageGaitCycle,TAUsDESIRED(:,1),'r','LineWidth',2);
hold on
p2 = plot(percentageGaitCycle,TAUs(:,1),'b','LineWidth',2);
xlabel('Gait Cycle [%]'); ylabel({'Hip Moment [Nm]';'extension    flexion'});
p3 = plot(percentageGaitCycle(1),TAUsDESIRED(1,1),'.k'); % HCR
text(percentageGaitCycle(1)+0.5,TAUsDESIRED(1,1)+4,'HCR');
p4 = plot(percentageGaitCycle(end),TAUsDESIRED(end,1),'.k'); % HCR
text(percentageGaitCycle(end)-6,TAUsDESIRED(end,1)+4,'HCR');
legend([p1 p2],'Desired','ExoNET');
box off

subplot(2,1,2)
p1 = plot(percentageGaitCycle,TAUsDESIRED(:,2),'r','LineWidth',2);
hold on
p2 = plot(percentageGaitCycle,TAUs(:,2),'b','LineWidth',2);
xlabel('Gait Cycle [%]'); ylabel({'Knee Moment [Nm]';'flexion    extension'});
p3 = plot(percentageGaitCycle(1),TAUsDESIRED(1,2),'.k'); % HCR
text(percentageGaitCycle(1)+0.5,TAUsDESIRED(1,2)-2,'HCR');
p4 = plot(percentageGaitCycle(end),TAUsDESIRED(end,2),'.k'); % HCR
text(percentageGaitCycle(end)-6,TAUsDESIRED(end,2)+4,'HCR');
legend([p1 p2],'Desired','ExoNET');
box off

end