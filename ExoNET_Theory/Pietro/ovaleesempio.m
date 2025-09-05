% Define the center and axis lengths
center = [1, 2, 3];  % Center of the oval (x, y, z)
xmag = 3;  % Semi-axis length along x-axis
ymag = 2;  % Semi-axis length along y-axis
zmag = 1;  % Semi-axis length along z-axis

% Generate the parametric angles
theta = linspace(0, 2*pi, 50);  % Azimuth angle
phi = linspace(0, pi, 25);      % Polar angle

% Parametric equations for the ellipsoid
x = a * sin(phi') * cos(theta) + center(1);
y = b * sin(phi') * sin(theta) + center(2);
z = c * cos(phi') * ones(size(theta)) + center(3);

% Plot the 3D oval using fill3
figure;
hold on;
for i = 1:length(phi)-1
    for j = 1:length(theta)-1
        % Define vertices of a small patch
        vx = [x(i, j), x(i+1, j), x(i+1, j+1), x(i, j+1)];
        vy = [y(i, j), y(i+1, j), y(i+1, j+1), y(i, j+1)];
        vz = [z(i, j), z(i+1, j), z(i+1, j+1), z(i, j+1)];
        % Fill the patch
        fill3(vx, vy, vz, 'cyan', 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    end
end
hold off;

% Adjust view
axis equal;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('3D Oval using Fill3');
view(3); % 3D view
grid on;