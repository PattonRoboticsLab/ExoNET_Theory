%% Create Torque Profiles for Elbow Flexion
data = load('elbow_flexion_momentarmdata.txt');
phi = data(:,1);
g = 9.8;
forearm_weight = 1*g;%0.408233*g;
bicep_MA = data(:,2)%.*forearm_weight;
brd_MA = data(:,3)%.*forearm_weight;
brachialis_MA = data(:,4)%.*forearm_weight;
upper_arm_length = .28
forearm_length = .25
arm_weight = 1
%muscle_force = (arm_weight * 9.8 * (forearm_length/2))/moment_arm
muscle_force = []
torque = []
%moment_arm = bicep_MA


L = .25;
%MA = brd_MA;



overall = bicep_MA+brd_MA+brachialis_MA
plot(phi, bicep_MA,'b', phi, brd_MA,'r', phi, brachialis_MA,'g', phi, overall,'c')
ylabel('Moment Arm (mm)')
xlabel('Phi Angle (/phi)')
legend('Biceps','Brachioradialis','Brachialis','Elbow Flexion')
plot(phi, torque)

%y = @(p, phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); %arm for the force 
%spring = @(p, phi) sqrt(L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi)); %spring length