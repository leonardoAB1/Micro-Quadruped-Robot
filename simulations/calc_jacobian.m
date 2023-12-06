clc
clear all
close all
% Jacobiano Robot Cuadrupedo

syms theta_1 theta_2 theta_3 p q1 q2 alpha w d x y z real
q = [theta_1, theta_2, theta_3];

v = sqrt(d^2*(1+alpha^2+2*alpha*cos(theta_2)));
w = v * sin(theta_1-acos((d^2+v^2-(alpha*d)^2)/(2*d*v)));
gamma = theta_2-pi;

x = d*cos(theta_1) + d*alpha*cos(theta_1 + theta_2) + p;
y = d*sin(theta_1) + d*alpha*sin(theta_1 + theta_2) + w * sin(theta_3);
z = q1 + q2 + w * cos(theta_3);

% Compute the Jacobian matrix
J = simplify(jacobian([x; y; z], q), 10);

% Pretty print each element of the Jacobian matrix
counter = 0;
disp('Jacobian Matrix:');
for i = 1:size(J, 1)
    for j = 1:size(J, 2)
        fprintf('%d J(%d, %d) =', counter, i, j);
        disp(J(i, j));
        counter=counter+1;
    end
end

% Compute the determinant of the Jacobian matrix
det_J = simplify(det(J));

% Display the determinant
disp('Determinant of the Jacobian Matrix:');
disp(det_J);
