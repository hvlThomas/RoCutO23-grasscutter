%% Kinematic model
close all
import ETS3.*

L1=0.35;
L2=0.35;


robot_normal = Ty(L1) * Rz('q1') * Ty(L2) * Rz('q2');
robot_normal.plot([0,0])

%% DH Parameters
close all
import ETS3.*

j1 = Revolute('d', 0, 'a', L1, 'alpha', 0, 'offset', 0);
j2 = Revolute('d', 0, 'a', L2, 'alpha', 0, 'offset', 0);

robot = SerialLink([j1 j2],'name', 'my robot');

robot.plot([0,0])


%% fwd/inv kine
T_robot_1 = robot.fkine([0,pi/2,pi/2,0])
T_robot_2 = SE3(0,1,0) * SE3.rpy(0,0,90, 'deg')
qi = robot.ikine(T_robot_1, 'mask', [1 1 1 0 0 0])

robot.plot(qi)