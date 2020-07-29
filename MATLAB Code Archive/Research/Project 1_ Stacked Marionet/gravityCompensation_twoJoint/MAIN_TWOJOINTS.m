clear variables
close all
clc

n = 100;
L1 = .35;
L2 = .26;

phi = -pi/3;
phi1 = phi*ones(1, n);
phi2 = linspace(0, 2*pi/3, n);

sh_pos = [0; 0]; 
R_el = [cos(phi), -sin(phi); sin(phi), cos(phi)];
el_posBeta = sh_pos+([L1; 0]);
el_pos = sh_pos+(R_el*[L1; 0]);
R_wr = [cos(phi2(6)), -sin(phi2(6)); sin(phi2(6)), cos(phi2(6))];
wrist_positionBeta = el_pos+(R_wr*[L2; 0]);
wrist_position = el_pos+(R_wr*[L2; 0]);
beta = atan2(wrist_positionBeta(2), wrist_positionBeta(1));
% beta = zeros(1, n);
% for j = 1:n
%     
%     R_wr = [cos(phi2(j)), -sin(phi2(j)); sin(phi2(j)), cos(phi2(j))];
%     wrist_position = el_pos+(R_wr*[L2; 0]); 
%     beta(j) = atan2(wrist_position(2), wrist_position(1));
%     
% end


torqueArm = @(p, phi1, phi2) (p(2)*(L1^2+L2^2-2*L1*L2.*cos(pi-(phi2))).*...
    sin(p(1)-beta-phi1))./...
    ((L1^2+L2^2-2*L1*L2.*cos(pi-(phi2))).^2+p(2)^2-2*p(2)*(L1^2+L2^2-2*L1*L2.*cos(pi-(phi2))).*...
    cos(p(1)-beta-phi1));

%spring = @(p, phi1, phi2) ((L1^2+L2^2-2*L1*L2.*cos(pi-(phi2))).^2+p(2)^2-2*p(2)*(L1^2+L2^2-2*L1*L2.*cos(pi-(phi2))).*...
 %   cos(p(1)-beta-phi1));
%spring = @(p, phi1, phi2) (L2.*sin(phi2)./sin(beta)).^2+p(2)^2-2*p(2)*(L2.*sin(phi2)./sin(beta)).*cos(p(1)-beta-phi1);

p = [pi/3, .1, 500];
% y = torqueArm(p, phi1, phi2);
% plot(phi2, y, 'b');
% hold on
% sp = spring(p, phi1, phi2);
% plot(phi2, sp, 'r');
% grid on
% %xlabel('\Phi_1')
% xlabel('\Phi_2')
% ylabel('Torque Arm/Spring Length')

draw_systemModified(sh_pos, el_pos, wrist_position, R_el);
draw_marionetModified(p(2), p(1), p(3), sh_pos, wrist_position);
axis off
