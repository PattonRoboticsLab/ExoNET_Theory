% plotVectField:  plot the vactor field resulting from a 2-joint marionet
% patton 2019-Jan-14
% modified from second half of optimizedTorque_mixedDevice by carella

function plotVectField(PHIs,Bod,Pos,tau,Colr)
scaleF =.005;    % graphical scale factor for gforce vectors
scaleTau =.1;    % graphical scale factor toque pseudo-vectors

%% euclidian position
subplot(1,2,1); 
for i=1:size(PHIs,1)  % loop ea config
  eqWrF(i,:)=(     inv(jacobian(PHIs(i,:),Bod.L)')*tau(i,:)')'; % Force
  simpleArrow(Pos.wr(i,:),Pos.wr(i,:)+scaleF*eqWrF(i,:),Colr,1.75); hold on
end

simpleArrow([Pos.wr(1,1),Pos.wr(1,2)],[Pos.wr(1,1),Pos.wr(1,2)],'k',1.75); % for the legend
%text(Pos.wr(1,1),Pos.wr(1,2),'10 N'); % for the legend

axis image

%% plot in torque field in phi domain (this cheating -- not a true vector)
subplot(1,2,2); 
for i=1:size(PHIs,1), 
  simpleArrow(PHIs(i,:),PHIs(i,:)+scaleTau*tau(i,:),Colr,1.75); 
  plot(PHIs(i,1),PHIs(i,2),'.','color',Colr); hold on; % dot
end
xlabel('\phi _1'); ylabel('\phi _2'); title('Torques at positions'); 

%simpleArrow([-1.5, 2.1],[-1.5, 2.1]+[scaleTau, 0],'k',1.75); % for the legend
%text(-1.6,2.2,'1 Nm'); % for the legend




box off; axis image
