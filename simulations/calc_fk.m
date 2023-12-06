clc
%clear all
close all
% Analisis de Cinematica Robot Cuadrupedo

syms theta_1 theta_2 theta_3 p q1 q2 alpha d x y z real
q = [theta_1, theta_2, theta_3];% alpha beta gamma

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

%% Forward Kinematics
T_12=T(rotx(theta_3),zeros(3,1))*T(eye(3),[p 0 q1]');
T_23=T(rotz(theta_1),zeros(3,1))*T(eye(3),[0 d 0]');
T_3F=T(rotz(theta_2),zeros(3,1))*T(eye(3),[alpha*d 0 q2]');
T_14=simplify(T_12*T_23*T_3F, 15);

% Display the Homogeneous transform position
disp('Position:');
disp(T_14(1:3, 4));

%% Analytic Jacobian
% Extract the position and orientation components of the transformation matrix
p_BF = T_14(1:3, 4);  % Position vector
R_BF = T_14(1:3, 1:3);  % Rotation matrix

% Calculate the linear velocity Jacobian
Jv = simplify(jacobian(p_BF, q), 'Steps', 15);

% Display the Position Jacobian matrix
disp('Analytical Jacobian Matrix:');
pretty(Jv);

%% Singularities
% Calculate the determinant of the Jacobian matrix
Jv_det = det(Jv);
disp('Determinant of Jacobian Matrix:');
solution = simplify(Jv_det);
disp(solution);

%% Inverse Kinematics
%% Find theta_1, theta_2 and theta_3 given x, y, and z

% Define the desired end-effector position (x, y, z)
desired_position = [x; y; z];

% Calculate the end-effector rotation matrix
R_end_effector = T_14(1:3, 1:3);

% Extract the position part of T_14
end_effector_position = T_14(1:3, 4);

eq1 = T_14(1, 4) == x;
eq2 = T_14(2, 4) == y;
eq3 = T_14(3, 4) == z;

solution = solve([eq1, eq2, eq3], [x, y, z]);

disp('Solution FK:');
disp(['x = ', char(simplify(solution.x))]);
disp(['y = ', char(simplify(solution.y, 10))]);
disp(['z = ', char(simplify(solution.z))]);
