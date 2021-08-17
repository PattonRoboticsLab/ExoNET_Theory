% ***********************************************************************
% Plot the torque vector field
% ***********************************************************************

function plotVectFieldLeg(PHIs,BODY,Position,tau,Color)

scaleF = 0.005; % graphic scale factor for force vectors
scaleTau = 0.2; % graphic scale factor for torque pseudo-vectors


%% Plot the Equivalent Force at the ankle positions
% % subplot(1,2,1)
% % for i = 1:size(PHIs,1)
% %     eqAnkleForce(i,:) = (inv(jacobian(PHIs(i,:),BODY.Lengths)')*tau(i,:)')'; % force at the ankle
% %     simpleArrow(Position.ankle(i,:),Position.ankle(i,:)+scaleF*eqAnkleForce(i,:),Color,1.75);
% %     hold on
% % end
% % text(0.1,Position.ankle(1,2)-0.1,'   ','Color',Color)
% % axis image


%% Plot the Torque Field in the PHIs Domain (this cheating -- not true vectors)
subplot(1,2,2)
for i = 1:size(PHIs,1)
    simpleArrow(PHIs(i,:),PHIs(i,:)+scaleTau*tau(i,:),Color,1.75);
    plot(PHIs(i,1),PHIs(i,2),'.','Color',Color) % dots
    hold on
end
xlabel('Hip Angle [deg]'); ylabel('Knee Angle [deg]'); title('Torques at angle positions');
simpleArrow([-2.8, 70],[-3, 70]+[scaleTau*10, 0],'k',1.75); % for the legend
text(-4,71.5,'10 Nm'); % for the legend
% plot(PHIs(1,1),PHIs(1,2),'.k'); plot(PHIs(70,1),PHIs(70,2),'.k'); % TOR
% text(PHIs(1,1)-1.3,PHIs(1,2)+2.1,'TOR');
% plot(PHIs(28,1),PHIs(28,2),'.k'); plot(PHIs(97,1),PHIs(97,2),'.k'); % HCR
% text(PHIs(28,1)-6,PHIs(28,2),'HCR');
box off
axis image

end