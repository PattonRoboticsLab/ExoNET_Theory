function draw_system(shoulder_pos,elbow_pos,wrist_pos)

global fig2

head_position = [0;.4];
bas_position = [0;-.5];
nose_position1 = [.11,.18];
nose_position2 = [0;.18];
eye_pos = [.05; .23];

dist_headbas = [head_position,bas_position];
dist_se = [shoulder_pos, elbow_pos];
dist_noshead = [head_position, nose_position1];
dist_ew = [elbow_pos,wrist_pos];
dist_nose = [nose_position1,nose_position2];

figure();
plot(dist_headbas(1,:),dist_headbas(2,:),'m','linewidth',5);
grid on
hold on
plot(dist_se(1,:),dist_se(2,:),'m','linewidth',5);
hold on
plot(dist_ew(1,:),dist_ew(2,:),'m','linewidth',5);
hold on
plot(dist_noshead(1,:),dist_noshead(2,:),'m','linewidth',5);
hold on
scatter(head_position(1),head_position(2),2000,'m','filled');
hold on
scatter(elbow_pos(1),shoulder_pos(2),100,'k','filled');
hold on
scatter(wrist_pos(1),wrist_pos(2),100,'k','filled');
hold on
scatter(eye_pos(1),eye_pos(2),'k','filled');
end
