% plotVectField:  plot the vactor field resulting from a 2-joint marionet
% patton 2019-Jan-14
% modified from second half of optimizedTorque_mixedDevice by carella

function plotVectField_1(PHIs,Bod,Pos,tau,Colr)
scaleF =.005;    % graphical scale factor for gforce vectors
scaleTau =.1;    % graphical scale factor toque pseudo-vectors

%% euclidian position
subplot(1,2,1); % figure(1);
for i=1:size(PHIs,1)  % loop ea config
  eqWrF=(    inv (jacobian(PHIs(i,:),Bod.L)') * tau(i,:)')'; % Force
   simpleArrow_2(Pos.wr(i,:),Pos.wr(i,:)+scaleF*eqWrF,Colr,3); hold on
   plot(Pos.wr(i,1),Pos.wr(i,2),'.','Color',Colr); % dot
end

plot(.1-[0 -scaleF*10],Pos.wr(1,2)-[0 0]-.1,Colr); % FOR LEGEND
text(.1,Pos.wr(1,2)-.1,'   ','Color',Colr);
axis image

%% plot in torque field in phi domain (this cheating -- not a true vector)
subplot(1,2,2); % figure(2)
% plot(0,0,'.'); hold on
for i=1:size(PHIs,1), 
  simpleArrow_2(PHIs(i,:),PHIs(i,:)+scaleTau*tau(i,:),Colr,3); 
  plot(PHIs(i,1),PHIs(i,2),'.','color',Colr); hold on; % dot
end
xlabel('\phi _1'); ylabel('\phi _2'); title('Torques at positions'); 
plot(PHIs(1,1)-[0 -scaleTau*10],PHIs(1,2)-[0 0]-.1,Colr); % FOR LEGEND
text(PHIs(1,1),PHIs(1,2)-.1,'       ','Color',Colr)
box off; axis image

%%
% subplot(3,1,3); % figure(3);
% plot3(PHIs(:,1),PHIs(:,2),tau(:,1),[Colr '+']); 
% hold on; box off; xlabel('\phi _1');  ylabel('\tau');
% plot3(PHIs(:,1),PHIs(:,2),tau(:,2),[Colr 'o']);

% 
% subplot(6,1,6); % figure(4);
% plot(PHIs(:,2),tau(:,2),[Colr '.']); 
% hold on; box off; xlabel('\phi _2');  ylabel('\tau _2');