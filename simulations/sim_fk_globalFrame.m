clc
clear all;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%Forward Kinematics%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Simulation%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Mechanism constants
a= 3/100;
b= 10/100;
c= 3/100;
d= 10/100;
alpha= 0.707107;

%% Robot design constants
a1= 25.5/1000;
a2= 6.5/1000;
p= 17.25/1000;
r_x=36/1000;
r_y=62.5/1000;
gamma= @(theta_2)theta_2-pi;
w= @(theta_2) sqrt(d^2*(1+alpha^2-2*alpha*cos(gamma(theta_2))));

%% Kinematic Equations Left Front Leg
x = @(theta_1, theta_2, theta_3) -d*cos(theta_1) - d*alpha*cos(theta_1 + theta_2) + (p+r_x);
y = @(theta_1, theta_2, theta_3) (a1 + a2 + r_y) + w(theta_2)*cos(theta_3);
z = @(theta_1, theta_2, theta_3) - d*sin(theta_1) + ...
    - alpha*d*sin(theta_1 + theta_2) - w(theta_2)*sin(theta_3);

%% Kinematic Equations Right Front Leg
x_rf = @(theta_1, theta_2, theta_3) -d*cos(theta_1) - d*alpha*cos(theta_1 + theta_2) + (p+r_x);
y_rf = @(theta_1, theta_2, theta_3) -(a1 + a2 + r_y) + w(theta_2)*cos(pi-theta_3);
z_rf = @(theta_1, theta_2, theta_3) - d*sin(theta_1) + ...
    - alpha*d*sin(theta_1 + theta_2) - w(theta_2)*sin(pi-theta_3);

%% Kinematic Equations Right Back Leg
x_rb = @(theta_1, theta_2, theta_3) -d*cos(theta_1) - d*alpha*cos(theta_1 + theta_2) - (p+r_x);
y_rb = @(theta_1, theta_2, theta_3) -(a1 + a2 + r_y) + w(theta_2)*cos(pi-theta_3);
z_rb = @(theta_1, theta_2, theta_3) - d*sin(theta_1) + ...
    - alpha*d*sin(theta_1 + theta_2) - w(theta_2)*sin(pi-theta_3);

%% Kinematic Equations Left Back Leg
x_lb = @(theta_1, theta_2, theta_3) -d*cos(theta_1) - d*alpha*cos(theta_1 + theta_2) - (p+r_x);
y_lb = @(theta_1, theta_2, theta_3) (a1 + a2 + r_y) + w(theta_2)*cos(theta_3);
z_lb = @(theta_1, theta_2, theta_3) - d*sin(theta_1) + ...
    - alpha*d*sin(theta_1 + theta_2) - w(theta_2)*sin(theta_3);

%% Plot Robot Workspace range between 0 and pi

% Define the range for each joint angle
theta_1_range = linspace(0.31677726, 1.4529866,20);
theta_2_range = linspace(0.436332, pi, 20);
theta_3_range = linspace(0, pi/2, 20);

% Calculate data points outside the loops
xy_points = zeros(length(theta_1_range), length(theta_2_range), 2);
zy_points = zeros(length(theta_2_range), length(theta_3_range), 2);
xz_points = zeros(length(theta_1_range), length(theta_3_range), 2);
xyz_points = zeros(length(theta_1_range), length(theta_2_range), length(theta_3_range), 3);
xyz_rf_points = zeros(length(theta_1_range), length(theta_2_range), length(theta_3_range), 3);
xyz_rb_points = zeros(length(theta_1_range), length(theta_2_range), length(theta_3_range), 3);
xyz_lb_points = zeros(length(theta_1_range), length(theta_2_range), length(theta_3_range), 3);

for i = 1:length(theta_1_range)
    for j = 1:length(theta_2_range)
        for k = 1:length(theta_3_range)
            % Calculate the corresponding x, y, and z coordinates
            current_x = x(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            current_y = y(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            current_z = z(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            
            current_x_rf = x_rf(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            current_y_rf = y_rf(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            current_z_rf = z_rf(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            
            current_x_rb = x_rb(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            current_y_rb = y_rb(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            current_z_rb = z_rb(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            
            current_x_lb = x_lb(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            current_y_lb = y_lb(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            current_z_lb = z_lb(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            
            % Save data points for xy-plane, zy-plane, and 3D space
            xy_points(i, j, :) = [current_x, current_y];
            zy_points(j, k, :) = [current_z, current_y];
            xz_points(i, k, :) = [current_x, current_z];
            xyz_points(i, j, k, :) = [current_x, current_y, current_z];
            xyz_rf_points(i, j, k, :) = [current_x_rf, current_y_rf, current_z_rf];
            xyz_rb_points(i, j, k, :) = [current_x_rb, current_y_rb, current_z_rb];
            xyz_lb_points(i, j, k, :) = [current_x_lb, current_y_lb, current_z_lb];
        end
    end
end

%Points to plot robot drawing
random_theta_1 = theta_1_range(randi(length(theta_1_range)));
random_theta_2 = theta_2_range(randi(length(theta_2_range)));
random_theta_3 = theta_3_range(randi(length(theta_3_range)));

x00=r_x;
y00=r_y;
z00=0;

x0 = p+x00;
y0 = a1+a2+y00;
z0 = 0+z00;
x1 = x0-d*cos(random_theta_1);
z1 = z0-d*sin(random_theta_1);

x2 = x(random_theta_1, random_theta_2, random_theta_3);
y2 = y(random_theta_1, random_theta_2, random_theta_3);
z2 = z(random_theta_1, random_theta_2, random_theta_3);

y1 = ((z1-z0)/((z2-z0)/(y2-y0)))+y0;

%mechanism points
x_bc = x1-c*cos(atan2((z2-z1),(x2-x1)));
z_bc = ((z2-z1)/(x2-x1))*(x_bc-x1)+z1;
y_bc = ((y2-y0)/(z2-z0))*(z_bc-z0)+y0;

x_ab= x_bc+b*cos(random_theta_1);
z_ab= z_bc+b*sin(random_theta_1);
y_ab = ((y2-y0)/(z2-z0))*(z_ab-z0)+y0;

% Reshape the data for 3D plotting
xyz_points_reshaped = reshape(xyz_points, [], 3);
xyz_rf_points_reshaped = reshape(xyz_rf_points, [], 3);
xyz_rb_points_reshaped = reshape(xyz_rb_points, [], 3);
xyz_lb_points_reshaped = reshape(xyz_lb_points, [], 3);

% Plot subplots
figure;

% XY-plane
subplot(1, 2, 1);
hold on;
plot(xz_points(:, :, 1), xz_points(:, :, 2), 'b.');
title('XZ-plane');
xlabel('X-axis');
ylabel('Z-axis');
grid on;
axis square;

%graph robot leg
plot(0, 0, 'k.');
plot(x00, z00, 'k.');
plot(x0, 0, 'k.');
plot([0 x0], [0 0], 'k');
plot([x0 x0], [0 z0], 'k');

plot(x0, z0, 'k.');
plot(x1, z1, 'r.');
plot(x2, z2, 'r.');
plot([x0 x1], [z0 z1], 'r');
plot([x1 x2], [z1 z2], 'r');

plot(x_bc, z_bc, 'm.');
plot(x_ab, z_ab, 'm.');
plot([x1 x_bc], [z1 z_bc], 'm');
plot([x0 x_ab], [z0 z_ab], 'm');
plot([x_ab x_bc], [z_ab z_bc], 'm');

hold off;

% YZ-plane
subplot(1, 2, 2);
hold on;
plot(xyz_points_reshaped(:, 1), xyz_points_reshaped(:, 3), 'b.');
title('YZ-plane');
xlabel('Y-axis');
ylabel('Z-axis');
grid on;
axis square;

%graph robot leg
plot(0, 0, 'k.');
plot(y00, z00, 'k.');
plot([0 y0], [0 z0], 'k');

plot(y0, z0, 'k.');
plot(y1, z1, 'r.');
plot(y2, z2, 'r.');
plot([y0 y1], [z0 z1], 'r');
plot([y1 y2], [z1 z2], 'r');

plot(y_ab, z_ab, 'm.');
plot([y0 y_ab], [z0 z_ab], 'm');

hold off;

% 3D space
%subplot(1, 3, 3);
figure(2);
hold on;

plot3(xyz_points_reshaped(:, 1), xyz_points_reshaped(:, 2), xyz_points_reshaped(:, 3), 'b.');
plot3(xyz_rf_points_reshaped(:, 1), xyz_rf_points_reshaped(:, 2), xyz_rf_points_reshaped(:, 3), 'b.');
plot3(xyz_rb_points_reshaped(:, 1), xyz_rb_points_reshaped(:, 2), xyz_rb_points_reshaped(:, 3), 'b.');
plot3(xyz_lb_points_reshaped(:, 1), xyz_lb_points_reshaped(:, 2), xyz_lb_points_reshaped(:, 3), 'b.');
title('3D Space');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
grid on;

view(3);

%graph robot leg

plot3(0, 0, 0, 'k.');
plot3(x00, 0, 0, 'k.');
plot3(x0, y00, z00, 'k.');
plot3([0 x00], [0 0], [0 0], 'k');
plot3([x00 x00], [0 y00], [0 z00], 'k');
plot3([x00 x0], [y00 y00], [z00 z00], 'k');
plot3([x0 x0], [y00 y0], [z00 z0], 'k');
plot3([0 x00], [0 y00], [0 z00], 'g');

plot3(x0, y0, z0, 'k.');
plot3(x1, y1, z1, 'r.');
plot3(x2, y2, z2, 'r.');
plot3([x0 x1], [y0 y1], [z0 z1], 'r');
plot3([x1 x2], [y1 y2], [z1 z2], 'r');

plot3(x_ab, y_ab, z_ab, 'm.');
plot3(x_bc, y_bc, z_bc, 'm.');
plot3([x1 x_bc], [y1 y_bc], [z1 z_bc], 'm');
plot3([x0 x_ab], [y0 y_ab], [z0 z_ab], 'm');
plot3([x_ab x_bc], [y_ab y_bc], [z_ab z_bc], 'm');

hold off;
