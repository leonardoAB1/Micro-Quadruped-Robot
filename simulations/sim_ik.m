clc
%clear all;
close all;

%Inverse Kinematics Simulation

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

%% Inverse Kinematic Equations
w_ = @(gamma) sqrt(d^2*(1+alpha^2-2*alpha*cos(gamma)));
theta_2 = @(x, y, z, gamma) pi + gamma;
theta_3 = @(x, y, z, gamma) atan2(sqrt(1-((z-a1-a2)/w_(gamma))^2),((z-a1-a2)/w_(gamma)));
theta_1 = @(x, y, z, gamma) atan2(y-w_(gamma)*sin(theta_3(x, y, z, gamma)),x-p)...
    - atan2(alpha*sin(theta_2(x, y, z, gamma)),1+alpha*cos(theta_2(x, y, z, gamma)));
