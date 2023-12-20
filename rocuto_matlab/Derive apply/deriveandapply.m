%% Derive and apply inverse kinematics to twolinkrobot
%mathworks.com/help/symbolic/derive-and-apply-inverse-kinematics-to-robot-arm.html

% 1. Define geometric parameters
syms L_1 L_2 theta_1 theta_2 XE YE

L1 = 350
L2 = 350

%Define x and y coordinates of the endeffector as function of theta_1 and
%theta_2
XE_RHS = L_1*cos(theta_1) + L_2*cos(theta_1+theta_2)
YE_RHS = L_1*sin(theta_1) + L_2*sin(theta_1+theta_2)

%Convert the symbolic ekspression into MATLAB functions
XE_MLF = matlabFunction(XE_RHS,'vars',[L_1 L_2 theta_1 theta_2]);
YE_MLF = matlabFunction(YE_RHS,'vars',[L_1 L_2 theta_1 theta_2]);

% calculate and visualize fwd kine
t1_degs_row = linspace(-90,90,100);
t2_degs_row = linspace(-270,270,100);
[tt1_degs,tt2_degs] = meshgrid(t1_degs_row,t2_degs_row);

tt1_rads = deg2rad(tt1_degs);
tt2_rads = deg2rad(tt2_degs);

X_mat = XE_MLF(L1,L2,tt1_rads,tt2_rads);
Y_mat = YE_MLF(L1,L2,tt1_rads,tt2_rads);

plot_XY_given_theta_2dof(tt1_degs,tt2_degs,X_mat,Y_mat,(L1+L2))