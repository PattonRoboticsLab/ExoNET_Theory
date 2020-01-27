% ***********************************************************************
% Plot the vector field resulting from a 2-joint Marionet
% ***********************************************************************

function plotVectField(PHIs,BODY,Position,tau,Color)

scaleF = 0.005; % graphic scale factor for force vectors
scaleTau = 0.1; % graphic scale factor for torque pseudo-vectors


%% Plot the Equivalent Force at the ankle positions
subplot(1,2,1)
for i = 1:size(PHIs,1)
    eqAnkleForce(i,:) = (inv(jacobian(PHIs(i,:),BODY.Lengths)')*tau(i,:)')'; % force at the ankle
    simpleArrow(Position.ankle(i,:),Position.ankle(i,:)+scaleF*eqAnkleForce(i,:),Color,1.75);
    hold on
end
text(0.1,Position.ankle(1,2)-0.1,'   ','Color',Color)
axis image


%% Plot in Torque Field in PHI Domain (this cheating -- not a true vector)
subplot(1,2,2)
for i = 1:size(PHIs,1)
    simpleArrow(PHIs(i,:),PHIs(i,:)+scaleTau*tau(i,:),Color,1.75);
    plot(PHIs(i,1),PHIs(i,2),'.','Color',Color) % dots
    hold on
end
xlabel('\phi _1'); ylabel('\phi _2'); title('Torques at positions');
plot(PHIs(1,1)-[0, -scaleTau*10],PHIs(1,2)-[0, 0]-0.1,'Color',Color) % for the legend
text(PHIs(1,1),PHIs(1,2)-0.1,'       ','Color',Color)
box off
axis image

end