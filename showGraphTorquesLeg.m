% ***********************************************************************
% Plot the ExoNET torques superimposed on the desired torques
% ***********************************************************************

figure
subplot(2,1,1)
p1 = plot(percentageGaitCycle,TAUsDESIRED(:,1),'r','LineWidth',2);
hold on
p2 = plot(percentageGaitCycle,TAUs(:,1),'b','LineWidth',2);
xlabel('Gait Cycle [%]'); ylabel({'Hip Moment [Nm]';'extension    flexion'});
legend([p1 p2],'Desired','ExoNET');

subplot(2,1,2)
p1 = plot(percentageGaitCycle,TAUsDESIRED(:,2),'r','LineWidth',2);
hold on
p2 = plot(percentageGaitCycle,TAUs(:,2),'b','LineWidth',2);
xlabel('Gait Cycle [%]'); ylabel({'Knee Moment [Nm]';'flexion    extension'});
legend([p1 p2],'Desired','ExoNET');

