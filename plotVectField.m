% plotVectField:  plot the vactor field resulting from a 2-joint marionet
% patton 2019-Jan-14
% modified from second half of optimizedTorque_mixedDevice by carella

function plotVectField(PHIs,Bod,Pos,tau,Colr)
scaleF =.002;    % graphical scale factor for gforce vectors
scaleTau =.02;    % graphical scale factor toque pseudo-vectors

%% euclidian position domain larde o neh side
subplot(1,2,1);hold on
for i=1:size(PHIs,1)  % loop for each point 
  eqWrF=(    inv (jacobian(PHIs(i,:),Bod.L)') * tau(i,:)')'; % Force
  simpleArrow(Pos.wr(i,:),Pos.wr(i,:)+scaleF*eqWrF,Colr,1); hold on
  %plot(Pos.wr(i,1),Pos.wr(i,2),'.','color',Colr); % dot
end
plot(.1-[0 -scaleF*10],Pos.wr(1,2)-[0 0]-.1,Colr); % scale ref
text(.1,Pos.wr(1,2)-.1,'   10 N','Color',Colr);
axis image

%% plot in torque field in phi domain (this cheating -- not a true vector)
subplot(2,2,2); 
for i=1:size(PHIs,1), 
  simpleArrow(PHIs(i,:),PHIs(i,:)+scaleTau*tau(i,:),Colr,1); hold on; 
  plot(PHIs(i,1),PHIs(i,2),'.','color',Colr); hold on; % dot
end
xlabel('\phi _1'); ylabel('\phi _2'); title('Torques at positions'); 
plot(PHIs(1,1)-[0 -scaleTau*10],PHIs(1,2)-[0 0]-.1,Colr); % FOR LEGEND
text(PHIs(1,1),PHIs(1,2)-.1,'       10 N*m','Color',Colr)
box off; axis image

%% force-length
subplot(2,2,4);  
plot(0,0,'k+');  % jimPlot
hold on; box off; xlabel('length');  ylabel('force');
