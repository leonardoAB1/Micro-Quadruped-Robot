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
gamma= @(theta_2)theta_2-pi;
w= @(theta_2) sqrt(d^2*(1+alpha^2-2*alpha*cos(gamma(theta_2))));

%% Kinematic Equations
x = @(theta_1, theta_2, theta_3) d*cos(theta_1) + d*alpha*cos(theta_1 + theta_2) - p;
y = @(theta_1, theta_2, theta_3) d*sin(theta_1) + ...
    d*alpha*sin(theta_1 + theta_2) + w(theta_2)*sin(theta_3);
z = @(theta_1, theta_2, theta_3) a1 + a2 + w(theta_2)*cos(theta_3);

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

for i = 1:length(theta_1_range)
    for j = 1:length(theta_2_range)
        for k = 1:length(theta_3_range)
            % Calculate the corresponding x, y, and z coordinates
            current_x = x(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            current_y = y(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            current_z = z(theta_1_range(i), theta_2_range(j), theta_3_range(k));
            
            % Save data points for xy-plane, zy-plane, and 3D space
            xy_points(i, j, :) = [current_x, current_y];
            zy_points(j, k, :) = [current_z, current_y];
            xz_points(i, k, :) = [current_x, current_z];
            xyz_points(i, j, k, :) = [current_x, current_y, current_z];
        end
    end
end

%Points to plot robot drawing
random_theta_1 = theta_1_range(randi(length(theta_1_range)));
random_theta_2 = theta_2_range(randi(length(theta_2_range)));
random_theta_3 = theta_3_range(randi(length(theta_3_range)));
x0 = -p;
y0 = 0;
z0 = a1+a2;
x1 = x0+d*cos(random_theta_1);
y1 = y0+d*sin(random_theta_1);

x2 = x(random_theta_1, random_theta_2, random_theta_3);
y2 = y(random_theta_1, random_theta_2, random_theta_3);
z2 = z(random_theta_1, random_theta_2, random_theta_3);

z1 = ((z2-z0)/(y2-y0))*(y1-y0)+z0;

%mechanism points
x_bc=x1-c*cos(atan2((y2-y1),(x2-x1)));
y_bc=((y2-y1)/(x2-x1))*(x_bc-x1)+y1;
z_bc = ((z2-z0)/(y2-y0))*(y_bc-y0)+z0;

x_ab= x_bc-b*cos(random_theta_1);
y_ab= y_bc-b*sin(random_theta_1);
z_ab = ((z2-z0)/(y2-y0))*(y_ab-y0)+z0;

% Plot subplots
figure;

% XY-plane
subplot(1, 3, 1);
hold on;
plot(xy_points(:, :, 1), xy_points(:, :, 2), 'b.');
title('XY-plane');
xlabel('X-axis');
ylabel('Y-axis');
grid on;
axis square;

%graph robot leg

plot(0, 0, 'k.');
plot(x0, 0, 'k.');
plot([0 x0], [0 0], 'k');
plot([x0 x0], [0 y0], 'k');

plot(x0, y0, 'k.');
plot(x1, y1, 'r.');
plot(x2, y2, 'r.');
plot([x0 x1], [y0 y1], 'r');
plot([x1 x2], [y1 y2], 'r');

plot(x_bc, y_bc, 'm.');
plot(x_ab, y_ab, 'm.');
plot([x1 x_bc], [y1 y_bc], 'm');
plot([x0 x_ab], [y0 y_ab], 'm');
plot([x_ab x_bc], [y_ab y_bc], 'm');

hold off;

% ZY-plane
subplot(1, 3, 2);
hold on;
plot(zy_points(:, :, 1), zy_points(:, :, 2), 'b.');
title('ZY-plane');
xlabel('Z-axis');
ylabel('Y-axis');
grid on;
axis square;

%graph robot leg

plot(0, 0, 'k.');
plot([0 z0], [0 y0], 'k');

plot(z0, y0, 'k.');
plot(z1, y1, 'r.');
plot(z2, y2, 'r.');
plot([z0 z1], [y0 y1], 'r');
plot([z1 z2], [y1 y2], 'r');

plot(z_ab, y_ab, 'm.');
plot([z0 z_ab], [y0 y_ab], 'm');

hold off;

% 3D space
subplot(1, 3, 3);
hold on;
% Reshape the data for 3D plotting
xyz_points_reshaped = reshape(xyz_points, [], 3);
plot3(xyz_points_reshaped(:, 1), xyz_points_reshaped(:, 2), xyz_points_reshaped(:, 3), 'b.');
title('3D Space');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
grid on;
axis square;
view(3);

%graph robot leg

plot3(0, 0, 0, 'k.');
plot3(x0, 0, 0, 'k.');
plot3([0 x0], [0 0], [0 0], 'k');
plot3([x0 x0], [0 y0], [0 z0], 'k');

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

fig = gcf;  % Get current figure handle
fig.Position(3) = fig.Position(3) * 2;  % Increase figure width

%%%%%%%%%%%%%%%%%%%%%%%%%%Inverse Kinematics%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Simulation%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Inverse Kinematic Equations
w_ = @(gamma) sqrt(d^2*(1+alpha^2-2*alpha*cos(gamma)));
theta_2 = @(x, y, z, gamma) pi + gamma;
theta_3 = @(x, y, z, gamma) atan2(sqrt(1-((z-a1-a2)/w_(gamma))^2),((z-a1-a2)/w_(gamma)));
theta_1 = @(x, y, z, gamma) atan2(y-w_(gamma)*sin(theta_3(x, y, z, gamma)),x+p)...
    - atan2(alpha*sin(theta_2(x, y, z, gamma)),1+alpha*cos(theta_2(x, y, z, gamma)));

% Calculate the inverse kinematics for each joint
computed_theta_1 = theta_1(x2, y2, z2, gamma(random_theta_2));
computed_theta_2 = theta_2(x2, y2, z2, gamma(random_theta_2));
computed_theta_3 = theta_3(x2, y2, z2, gamma(random_theta_2));

% Check computed thetas for each joint
decimal_places=5;
if isequal(round(random_theta_1, decimal_places), round(computed_theta_1, decimal_places))
    disp('Theta 1 Inverse Kinematics Works Fine.');
else
    disp('Theta 1 Inverse Kinematics Error.');
    disp(['Expected Value: ', num2str(random_theta_1)]);
    disp(['Computed Value: ', num2str(computed_theta_1)]);
end

if isequal(round(random_theta_2, decimal_places), round(computed_theta_2, decimal_places))
    disp('Theta 2 Inverse Kinematics Works Fine.');
else
    disp('Theta 2 Inverse Kinematics Error.');
    disp(['Expected Value: ', num2str(random_theta_2)]);
    disp(['Computed Value: ', num2str(computed_theta_2)]);
end

if isequal(round(random_theta_3, decimal_places), round(computed_theta_3, decimal_places))
    disp('Theta 3 Inverse Kinematics Works Fine.');
else
    disp('Theta 3 Inverse Kinematics Error.');
    disp(['Expected Value: ', num2str(random_theta_3)]);
    disp(['Computed Value: ', num2str(computed_theta_3)]);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%Full Quadruped Robot%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Simulation%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Matrices de rotaciones x, y, z
rotx = @(q) [1,      0,       0; ...
             0, cos(q), -sin(q); ...
             0, sin(q),  cos(q)];

roty = @(q) [cos(q), 0, sin(q); ...
              0,     1,      0; ...
            -sin(q), 0, cos(q)];

rotz = @(q) [cos(q), -sin(q), 0; ...
             sin(q),  cos(q), 0; ...
               0,       0,    1];

%% Homogeneous Transformation
T = @(C, vector) [ C     vector;
                  zeros(1,3) 1];
              
%% Move base leg to each position
T_rotx= @(angle) T(rotx(angle),zeros(3,1));
T_roty= @(angle) T(roty(angle),zeros(3,1));
T10=T_rotx(pi)*T(eye(3),[145/2 210/2 0]');
T02=T_rotx(pi)*T(eye(3),[-145/2 210/2 0]');
T30=T_rotx(pi)*T(eye(3),[145/2 -210/2 0]');
T04=T_rotx(pi)*T(eye(3),[-145/2 -210/2 0]');

p00=[0 0 0]';
p0=[x0 y0 z0]';
p1=[x1 y1 z1]';
p2=[x2 y2 z2]';
p_ab=[x_ab y_ab z_ab]';
p_bc=[x_bc y_bc z_bc]';

points=[p00 p0 p1 p2 p_ab p_bc];

% Create matrices to store transformed points
transformed_points_T10 = zeros(size(points));
transformed_points_T02 = zeros(size(points));
transformed_points_T30 = zeros(size(points));
transformed_points_T04 = zeros(size(points));

% Apply transforms to each point
for i = 1:size(points, 2)
    result= T10 * [points(:, i); 1];
    transformed_points_T10(:, i) = result(1:3);
    result= T02 * [points(:, i); 1];
    transformed_points_T02(:, i) = result(1:3);
    result= T30 * [points(:, i); 1];
    transformed_points_T30(:, i) = result(1:3);
    result= T04 * [points(:, i); 1];
    transformed_points_T04(:, i) = result(1:3);
end

%Plot the transformed points in 3D space
figure;
% Set axis limits to achieve a space size of 150 on each side of 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot3([transformed_points_T10(1, 1) transformed_points_T02(1, 1)], ...
    [transformed_points_T10(2, 1) transformed_points_T02(2, 1)], ...
    [transformed_points_T10(3, 1) transformed_points_T02(3, 1)], 'k');
hold on;

plot3([transformed_points_T10(1, 1) transformed_points_T30(1, 1)], ...
    [transformed_points_T10(2, 1) transformed_points_T30(2, 1)], ...
    [transformed_points_T10(3, 1) transformed_points_T30(3, 1)], 'm');

plot3([transformed_points_T30(1, 1) transformed_points_T04(1, 1)], ...
     [transformed_points_T30(2, 1) transformed_points_T04(2, 1)], ...
     [transformed_points_T30(3, 1) transformed_points_T04(3, 1)], 'r');

plot3([transformed_points_T04(1, 1) transformed_points_T02(1, 1)], ...
    [transformed_points_T04(2, 1) transformed_points_T02(2, 1)], ...
    [transformed_points_T04(3, 1) transformed_points_T02(3, 1)], 'g');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plotRoboticLeg(transformed_points_T10);
plotRoboticLeg(transformed_points_T02);
plotRoboticLeg(transformed_points_T30);
plotRoboticLeg(transformed_points_T04);

grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
view(3);
hold off;

figure
hold on 
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
%plotRoboticLeg(transformed_points_T10);
plotRoboticLeg(transformed_points_T30);
grid on;
view(3);
hold off

figure
hold on 
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
plotRoboticLeg(transformed_points_T02);
plotRoboticLeg(transformed_points_T04);
view(3);
grid on;
hold off
    