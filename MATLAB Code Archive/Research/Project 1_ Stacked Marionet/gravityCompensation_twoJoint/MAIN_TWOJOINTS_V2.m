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
beta = zeros(1, n);
wrist_position = zeros(2, n);
for j = 1:n
    
    R_wr = [cos(phi2(j)), -sin(phi2(j)); sin(phi2(j)), cos(phi2(j))];
    wrist_position(:, j) = el_posBeta+(R_wr*[L2; 0]); 
    beta(j) = atan2(wrist_position(2, j), wrist_position(1, j));
    
end

p = [pi/4, .1];
angle = p(1);
radius = p(2);
R = [cos(-pi/2+angle), -sin(-pi/2+angle); sin(-pi/2+angle), cos(-pi/2+angle)];

mar_norm = R*[0; radius];
mar_pos = mar_norm+sh_pos;

spring_length1 = sqrt((wrist_position(2, :)-mar_pos(2)).^2+(wrist_position(1, :)-mar_pos(1)).^2);
spring2 = @(p, phi1, phi2) sqrt((L2.*sin(phi2)./sin(beta)).^2+p(2)^2-2*p(2)*(L2.*sin(phi2)./sin(beta)).*cos(p(1)-beta-phi1));

spring = @(p, phi1, phi2) sqrt((sqrt(L1^2+L2^2-2*L1*L2.*cos(pi-(phi2)))).^2+p(2)^2-2*p(2)*sqrt(L1^2+L2^2-2*L1*L2.*cos(pi-(phi2))).*...
    cos(p(1)-beta-phi1));

figure(1)
plot(phi2, spring_length1);



spring_length2 = spring2(p, phi1, phi2);
hold on
plot(phi2, spring_length2, 'r', 'linewidth', 2);
spring_length = spring(p, phi1, phi2);
plot(phi2, spring_length, 'g');
legend('Target')

torqueArm1 = @(p, phi1, phi2) (p(2)*(L1^2+L2^2-2*L1*L2.*cos(pi-(phi2))).*...
    sin(p(1)-beta-phi1))./...
    sqrt((L2.*sin(phi2)./sin(beta)).^2+p(2)^2-2*p(2)*(L2.*sin(phi2)./sin(beta)).*cos(p(1)-beta-phi1));
torqueArm2 = @(p, phi1, phi2) (p(2)*(L1^2+L2^2-2*L1*L2.*cos(pi-(phi2))).*...
    sin(p(1)-beta-phi1))./...
    spring_length1;
torqueArm = @(p, phi1, phi2) (p(2)*(L1^2+L2^2-2*L1*L2.*cos(pi-(phi2))).*...
    sin(p(1)-beta-phi1))./...
    sqrt((sqrt(L1^2+L2^2-2*L1*L2.*cos(pi-(phi2)))).^2+p(2)^2-2*p(2)*sqrt(L1^2+L2^2-2*L1*L2.*cos(pi-(phi2))).*...
    cos(p(1)-beta-phi1));

figure(2)
plot(phi2, torqueArm1(p, phi1, phi2), 'b','linewidth', 2)
hold on
plot(phi2, torqueArm2(p, phi1, phi2), 'r')
hold on
plot(phi2, torqueArm(p, phi1, phi2), 'g')
legend('Target')

figure(3)
H1 = L2.*sin(pi-phi2)./sin(beta);
H2 = sqrt(L1^2+L2^2-2*L1*L2.*cos(pi-phi2));
H3 = sqrt((wrist_position(2, :)-sh_pos(2)).^2+(wrist_position(1, :)-sh_pos(1)).^2);
plot(phi2, H1, 'b', 'linewidth', 2)
hold on
plot(phi2, H2, 'r')
hold on
plot(phi2, H3, 'g')