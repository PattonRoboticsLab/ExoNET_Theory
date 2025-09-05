clear all; close all; clc
%% Parameters to evaluate the torque on the shoulder
g = 9.81;         % Gravity constant
mass = 80;        % Mass in Kg
hand_weight       = (0.61/100) * mass * g;                % 0.226796*g;%(0.61/100)*Bod.M*g;   % from Winter's book
foreArm_weight    = (1.62/100) * mass * g;                % 0.408233*g;%(1.62/100)*Bod.M*g;
forearmHandWeight = foreArm_weight + hand_weight;         % Sum 'cause they're 1`
upperArm_weight   = (2.71/100) * mass * g;                % (0.453592+0.317515)*g;            %(2.71/100)*Bod.M*g;
Bod.weights       = [upperArm_weight, foreArm_weight, hand_weight]; % Use a variable to attach weights of the various parts of the body

%% Evaluation of the spring and attachment
L_rail = [0.15, 0.2];
Rail_origin = pindellaspring - [0, L_rail, 0];
Kspring = [0, 1]; K_max = 1000;
Lspring = [0, 1]; L_max = 1/2;

center_rotation = [0, 0, 0];    
npoints = 8;
theta = linspace( 0, theta, npoints );   % Rotating 90 degrees from resting position
perc = 0.9;            L1 = 0.28; 
axis_upperarm = perc * L1 * [cos(theta); zeros(1, length(theta)); sin(theta)];

           L2 = L1 + 0.25; % L2 is the length of the whole arm
axis_arm = L2 * [cos(theta); zeros(1, length(theta)); sin(theta)];

%% Create the point of the two CM for the evaluation
CM1 = L1 * 0.5 * [cos(theta); zeros(1, length(theta)); sin(theta)];
CM2 = L1 * [cos(theta); zeros(1, length(theta)); sin(theta)] + L2 * 0.45 * [cos(theta); zeros(1, length(theta)); sin(theta)];

%% Create the pins 
r = 0.05;       angleyz = 135; anglexz = 135;
alpha = deg2rad(angleyz);
beta = deg2rad(anglexz);
PinA = [r*cos(beta), r*cos(alpha), r*sin(alpha)*sin(beta)];

% Evaluate the angle PINA-Shoulder
% Angle_Pin = atan2(z)

% Create the pin with the end of the spring
gap = 0.03;     gap_arm = gap * [sin(theta); zeros(1, length(theta)); cos(theta)];
PinB = axis_upperarm + abs(gap_arm);

%% Create the recoil system
Pinstart = [-0.2+PinA(1); PinA(2); PinA(3)];
Recoil_cent = [Pinstart(1)/2; PinA(2); PinA(3)];
Recoil_radius = 0.025;   Recoil_angle = deg2rad(linspace(0, 40, npoints)); % For the Recoil
Pinleft = Recoil_cent -  Recoil_radius *[cos(Recoil_angle); zeros(1, length(Recoil_angle)); sin(Recoil_angle)];
Pinright = Recoil_cent +  Recoil_radius *[cos(Recoil_angle); zeros(1, length(Recoil_angle)); sin(Recoil_angle)];

%% Calculate initial spring length in resting position
spring_length_recoil = zeros(1, npoints);
spring_without = zeros(1, npoints);

initial_spring_length_recoil = norm(Pinleft(:,1)-Pinstart(:,1)) + norm(Pinright(:,1)-Pinleft(:,1)) + ...
                                norm(Pinright(:,1)-PinA') + norm(PinB(:,1) - PinA');

initial_spring_without = norm(PinB(:,1) - PinA')+ norm(PinA' - Pinstart);

% Define the spring constant (example value, adjust as needed)
k = 1000; % N/m
L0_recoil  = initial_spring_length_recoil(1) / 2; % Resting length

% Evaluate extension spring
% lbs/inch in N/m moltiply by 175.126
k_ext = 56.6*175.126; % N/m
L0_without = initial_spring_without(1); % Since the spring increase length I put it in the L0 conformation

figure(), hold on;
scatter3(Pinstart(1), Pinstart(2), Pinstart(3))
for i = 1:length(theta)

    spring_length_without(i) = norm(PinB(:,i) - PinA') + norm(PinA' - Pinstart);

    spring_length_recoil(i) =  norm(Pinleft(:,i)-Pinstart) + norm(Pinright(:,i)-Pinleft(:,i)) + ...
                                norm(Pinright(:,i)-PinA') + norm(PinB(:,i) - PinA');
    
    scatter3(PinB(1,i), PinB(2,i), PinB(3,i)); % Plot the pin

    % Plot the whole spring
    plot3([Pinstart(1), Pinleft(1,i)], [Pinstart(2), Pinleft(2,i)], [Pinstart(3), Pinleft(3,i)], 'r'); % Plot the spring
    plot3([Pinleft(1,i), Pinright(1,i)], [Pinleft(2,i), Pinright(2,i)], [Pinleft(3,i), Pinright(3,i)], 'r');
    plot3([PinA(1), Pinright(1,i)], [PinA(2), Pinright(2,i)], [PinA(3), Pinright(3,i)], 'r')
    plot3([PinA(1), PinB(1,i)], [PinA(2), PinB(2,i)], [PinA(3), PinB(3,i)], 'r')
    
    % Plotta il braccio
    plot3([center_rotation(1), axis_arm(1,i)], [center_rotation(2), axis_arm(2,i)], [center_rotation(3), axis_arm(3,i)], 'g'); % Plot the arm
    
end
title('Spring and Arm Positions at Different Angles'); axis equal; xlim([-0.29 0.4]);ylim([-0.1 0.25]);zlim([-0.25 0.2]);
xlabel('X Position'); ylabel('Y Position'); zlabel('Z Position'); view(3); hold off;

% Calculate the change in spring length relative to the initial length
delta_spring_length_recoil = spring_length_recoil - initial_spring_length_recoil;
delta_spring_without = spring_length_without - initial_spring_without;

% Calculate the tension in the spring
% spring_tension_recoil = 9.81 * (1.04 + 2.124 * ((spring_length_recoil-L0_recoil)/L0_recoil));
% spring_tension_without = 9.81 * (1.04 + 2.124 *((spring_length_without-L0_without)/L0_without));

spring_tension_recoil = k * (spring_length_recoil - L0_recoil); % Tension apllied to the recoil system

spring_tension_without = k * (spring_length_without - L0_without); % Evaluate the bunjee cord
extension_spring = k_ext * (spring_length_without - L0_without); % Evaluate the extension spring 

% Plot the length of the spring over angle
figure('Name','Lengths of the spring over angle')
plot(180/pi*theta, delta_spring_without,'LineWidth',2); hold on; xlabel('Angle (deg)'); ylabel('Spring length (m)');
plot(180/pi*theta, spring_length_without,'LineWidth',2); 
plot(180/pi*theta, spring_length_recoil,'LineWidth',2); 
plot(180/pi*theta, ones(1,npoints) * abs(max(delta_spring_without)-min(delta_spring_without)),'o');
legend('Delta spring','Spring length','Spring length recoil','Delta max')

% Plot the spring tension over time
figure('Name','Tensions over angle'), 
plot(180/pi*theta, spring_tension_without, '-o','LineWidth',2); hold on; title('Spring Tension Over angle'); xlabel('Angle (deg)'); ylabel('Spring Tension (N)');
plot(180/pi*theta, extension_spring, '-o','LineWidth',2); 
plot(180/pi*theta, spring_tension_recoil, '-o','LineWidth',2); 
legend ('Without recoil','Extension spring','Recoil system')

%% Calculate the torque using the cross product in 3D
torque = zeros(1, length(theta));
for i = 1:length(theta)

    rVect = PinB(:,i)'; % Position vector from center of rotation to PinB
    lVect = PinA; % Position of the spring on the shoulder
    Tdir = lVect - rVect; % Vector of tension element
    Tdist = norm(Tdir); % Magnitude: length, rotator to endpoint
    Tdir = Tdir / Tdist; % Tension direction vector
        
    % Torque exrcited from bunjee cord
    Tauvect_without = cross(rVect, spring_tension_without(i) * Tdir);
    Torque_without(i) = Tauvect_without(2);
    
    % Torque exrcited from ext spring on shoulder
    Tauvect_ext_spring = cross(rVect, extension_spring(i) * Tdir);
    Torque_ext_spring(i) = Tauvect_ext_spring(2);
    
    % Torque to deny of the shoulder
    Torque_Shoulder_tot = cross(CM1(:,i)', [0, 0, -upperArm_weight]) + cross(CM2(:,i)', [0 ,0, -forearmHandWeight]);
    Torque_Shoulder(i) = Torque_Shoulder_tot(2);

end

%% Parameters of the recoil spring
deflection_angle = 120 / 180 * pi; %120 gradi in rad
torque_max_torsion = 4; %Nm see what you have

%% Find the torque of the recoil system

for i=1:npoints
    
    rVect_right = Pinright(:,i) - Recoil_cent;
    Tdir_right = PinA' - Pinright(:,i); % Vector of tension element on the right    
    Tdist_right = norm(Tdir_right); % Magnitude: length, rotator to endpoint
    Tdir_right = Tdir_right / Tdist_right; % Tension direction vector

    Tau_recoil_right = cross(rVect_right, spring_tension_recoil(i) * Tdir_right); % Torque vector in 3D
    
    Tension_perp(i) = spring_tension_recoil(i) * cos(pi/2-Recoil_angle(i));

    Total_torque_recoil(i) = abs(Tau_recoil_right(2))*2;

    Force_recoil_needed(i) = Total_torque_recoil(i) / Recoil_radius; % Recoil_radius = norm(rVect_right)

    Torque_torsion_spring(i) = torque_max_torsion - Recoil_angle(i)/deflection_angle * torque_max_torsion;

    Force_recoil_exert(i) = Torque_torsion_spring(i) / Recoil_radius;

end

% Plot the torque over time
figure('Name','Torques over angle'), 
plot(180/pi*theta, Torque_without, '-o'); xlabel('Angle (deg)'); ylabel('Torque (Nm)'); hold on;
plot(180/pi*theta, Torque_ext_spring,'-o'); 
plot(180/pi*theta, Total_torque_recoil, '-o');
plot(180/pi*theta, Torque_Shoulder, '-o'); 
legend ('Without recoil', 'Extension spring',' Recoil system', 'Shoulder torque'); axis equal

% Plot the Forces on the Recoil system
figure('Name','Torque on recoil over angle')
plot(rad2deg(Recoil_angle), Total_torque_recoil,'-ok','LineWidth',2); hold on; xlabel('Angle (deg)'); ylabel('Torque (Nm)');
plot(rad2deg(Recoil_angle), Torque_torsion_spring ,'-ob','LineWidth',2)
legend('Torque recoil','Torsion spring torque')

figure('Name','Force on recoil over angle')
plot(rad2deg(Recoil_angle), Tension_perp,'-or','LineWidth',2); hold on; xlabel('Angle (deg)'); ylabel('Force (N)');
plot(rad2deg(Recoil_angle), Force_recoil_needed ,'hexagram','LineWidth',2)
plot(rad2deg(Recoil_angle), Force_recoil_exert ,'-om','LineWidth',2)
legend('Tension perp on recoil','Force needed','Force exerted recoil')
