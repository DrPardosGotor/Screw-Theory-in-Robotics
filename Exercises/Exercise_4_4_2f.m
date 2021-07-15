%% Screw Theory in Robotics
% An Illustrated and Practicable Introduction to Modern Mechanics
% by CRC Press
% © 2022 Jose M Pardos-Gotor
%
%% Ch4 - INVERSE KINEMATICS.
%
% Exercise 4.4.2f: ABB IRB120 RobotToolDown.
%
% The goal of this exercise is to compare the performance between
% ST24R "Screw Theory Toolbox for Robotics" by Dr. Pardos-Gotor.
% and
% RST "Robotics System Toolbox" by MATHWORKS
% for solving the Inverse Kinematics of the IRB120:
%
% For the ST24R we use:
% function ThetaOut = Fcn_IRB120_Kinematics_ToolDown(u)
% The inputs "u" are composed by the following vectors.
% "traXYZ" (3x1) desired translations for the TcP (noap - "p" goal).
% "rotXYZ" (3x1) desired rotations for TcP (noap - "noa" goal order X+Y+Z).
% "ti" (1x1) "Theta index" for choosing ONE out of the 8 possible results.
% "ThetaOut" (t1..t6)are the magnitudes solution for the Robot Joints1..6.
%
% For the RST we use:
% The Broyden-Fletcher-Goldfarb-Shanno (BFGS) gradient projection algorithm
% iterative, gradient-based optimization methods. 
%
% For checking the quality of this IK solution, this exercise has 3 steps:
% STEP1: Apply ForwardKinemats for the Robot for "whatever" Mag Theta1...6
% so getting a feasible TcP configuration (rot+tra) as Hst.
% STEP2: Calculate the IK solutions by ST24R and RST getting
% the magnitud Theta1...6. There can be up to 8 right solutionS
% STEP3: Test the different solutions applying ForwardKinemats to Robot
% and checking we get the same TcP POSE configuration (rot+tra) as Hst.
%
% Copyright (C) 2003-2021, by Dr. Jose M. Pardos-Gotor.
%
% This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB
% 
% ST24R is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published
% by the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ST24R is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Lesser General Public License
% along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2021/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% MATLAB Code
%
clear
clc
for i = 1:6
    Mag(i) = (rand-rand)*pi; % Magnitudes Theta1-Theta6 for testing.
end
%Mag = [0.1 0.1 0.1 0.1 0.1 0.1];
MagST24R = Mag;
%% Application "ST24R - Screw Theory Robotics" for Kinematics.
% Mechanical characteristics of the robot
po=[0;0;0]; pk=[0;0;0.290]; pr=[0;0;0.560];
pf=[0.302;0;0.630]; pp=[0.302;0;0.470];
% Joints TWISTS definition and TcP at home.
Twist = [0   -0.2900   -0.5600         0   -0.6300         0;
         0         0         0    0.6300         0    0.3020;
         0         0         0         0    0.3020         0;
         0         0         0    1.0000         0         0;
         0    1.0000    1.0000         0    1.0000         0;
    1.0000         0         0         0         0   -1.0000];
Hst0 =[-1 0  0 0.302;
        0 1  0 0;
        0 0 -1 0.47;
        0 0  0 1];
%
% STEP1: Apply ForwardKinemats to the Robot.
TwMag = [Twist; MagST24R]; % assign the rand values to joint Theta magnitudes.
noapST24R = ForwardKinematicsPOE(TwMag) * Hst0;
noap = noapST24R
%
% STEP2: Calculate the IK solutions Theta using the SCREW THEORY techniques
traXYZ = noapST24R(1:3,4);
rotXYZ = tform2eul(noapST24R,'XYZ');
tic;
ThetaST24R = Fcn_ST24R_IK_ABBIRB120_ToolD([traXYZ', rotXYZ])
tIKST24R = round(toc*1000,0);
time_IK_ST24R = ['Time to solve IK ST24R ', num2str(tIKST24R),' ms']
%Fcn_ST24R_IK_ABBIRB120_ToolD
% STEP3: Test the solution applying ForwardKinemats to Robot
% we choose only the first solution for ThetaOut
TwMag = [Twist; ThetaST24R(1,:)];
TcpST24R = ForwardKinematicsPOE(TwMag) * Hst0
%
%% Application "RST - Robotics System Toolbox - MATLAB" - for Kinematics.
%
% Mechanical characteristics of the ABB IRB120 Robot.
% ATTENTION! because for the RST standard the coordinate system of the base
% has always the "Z" axis in vertical direction as it uses D-H convention
% Therefore the configuration of the robot has other coordinates.
%
% Create a rigid body tree object to build the robot.
robot = robotics.RigidBodyTree;
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');
body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','revolute');
%
tformj1 = trvZ2tform(pk(3));
tformj2 = rotX2tform(-pi/2);
tformj3 = trvY2tform(pk(3)-pr(3));
tformj4 = trvY2tform(pr(3)-pf(3))*trvX2tform(pf(1))*rotY2tform(pi/2);
tformj5 = rotY2tform(-pi/2);
tformj6 = trvY2tform(pf(3)-pp(3))*rotX2tform(-pi/2)*rotZ2tform(pi);
%
setFixedTransform(jnt1,tformj1);
setFixedTransform(jnt2,tformj2);
setFixedTransform(jnt3,tformj3);
setFixedTransform(jnt4,tformj4);
setFixedTransform(jnt5,tformj5);
setFixedTransform(jnt6,tformj6);
%
body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
%
addBody(robot,body1,'base');
addBody(robot,body2,'body1');
addBody(robot,body3,'body2');
addBody(robot,body4,'body3');
addBody(robot,body5,'body4');
addBody(robot,body6,'body5');
%
% STEP1: Apply ForwardKinemats to the Robot.
% Verify that your robot was built properly by using the |showdetails| or
% showdetails(robot);
% show(robot);
% getTransform permits to evaluate the Forward Kinematics.
joints = {'jnt1','jnt2','jnt3','jnt4','jnt5','jnt6'};
MagRST = {Mag(1),Mag(2),Mag(3),Mag(4),Mag(5),Mag(6)};
FKRST = struct('JointName', joints,'JointPosition', MagRST);
noapRST = getTransform(robot,FKRST,'body6');
%
% STEP2: Calculate the IK solutions with Robtics System Toolbox
tic;
% noapRSTbase = getTransform(robot,FKRST,'body7','base');
weights = [1e-6,1e-6,1e-6,1e-6,1e-6,1e-6];
qinitial = robot.homeConfiguration;
IKRST = robotics.InverseKinematics('RigidBodyTree', robot);
IKRST.SolverParameters.MaxIterations= 15000;
[IKRSTsol,solinfo] = IKRST('body6',noapRST,weights,qinitial);
ThetaRST6 = {IKRSTsol(1).JointPosition; IKRSTsol(2).JointPosition;
            IKRSTsol(3).JointPosition; IKRSTsol(4).JointPosition;
            IKRSTsol(5).JointPosition; IKRSTsol(6).JointPosition}';
ThetaRST = cell2mat(ThetaRST6)
tIKRST = round(toc*1000,0);
time_IK_RST = ['Time to solve IK RST Toolbox ', num2str(tIKRST),' ms']
%
% STEP3: Test the solution applying ForwardKinemats to Robot
FKRST = struct('JointName', joints,'JointPosition', ThetaRST6);
TcpRST = getTransform(robot,FKRST,'body6')
%
%