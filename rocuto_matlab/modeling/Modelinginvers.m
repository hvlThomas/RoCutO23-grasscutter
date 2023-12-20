%% Modeling Inverse Kinematics in Robotic Arm
% mathworks.con/help/fuzzy/modeling-inverse-kinematics-in-a-robotic-arm
clc
clear

L1 = 350/1000; % measure in meter
L2 = 350/1000;

theta1 = 0:0.001:pi;
theta2 = -3*pi/4:0.001:3*pi/4;

[THETA1,THETA2] = meshgrid(theta1,theta2);

X = L1 * cos(THETA1) + L2 * cos(THETA1 + THETA2);
Y = L1 * sin(THETA1) + L2 * sin(THETA1 + THETA2);

data1 = [X(:) Y(:) THETA1(:)];
data2 = [X(:) Y(:) THETA2(:)];

yl=[0 inf];


plot(X(:),Y(:),'r');
% axis equal;
xlabel('X','FontSize',10)
ylabel('Y','FontSize',10)
title('X-Y coordinates for all theta1 and theta2 combinations in meter','FontSize',10)
ylim(yl);