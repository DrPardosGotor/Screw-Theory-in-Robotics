%% Screw Theory for Robotics - Inverse Kinematics.
% PUMA Robot.
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
% Copyright (C) 2003-2020, by Dr. Jose M. Pardos-Gotor.
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
% Revision 1.1  2020/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% E421a_ST24R_IK_PUMA_vsRST
%
clear
clc
for i = 1:6
    Mag(i) = (rand-rand)*pi; % Magnitudes Theta1-Theta6 for testing.
end
%Mag = [0.05 0.05 0.05 0.05 0.05 0.05];
MagST24R = Mag;
%% Application "ST24R - Screw Theory Robotics" for Kinematics.
%
% Mechanical characteristics of the IRB120 Robot:
po = [0; 0; 0]; pk = [0; 0.66; 0]; pr = [0; 1.092; 0]; pf = [0;1.524;0];
pp = [0;1.524;0.15];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Axis = [AxisY AxisX AxisX AxisY AxisX AxisZ];
Point = [pk pk pr pf pf pf];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Twist = zeros(6,6);
for i = 1:6
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvY2tform(pp(2))*trvZ2tform(pp(3));
%
% STEP1: Apply ForwardKinemats to the Robot.
TwMag = [Twist; MagST24R]; % assign the rand values to joint Theta magnitudes.
HstR = ForwardKinematicsPOE(TwMag);
noapST24R = HstR * Hst0
%
% STEP2: Calculate the IK solutions Theta using the SCREW THEORY techniques
traXYZ = noapST24R(1:3,4);
rotXYZ = tform2eul(noapST24R,'XYZ');
%
tic;
Theta = zeros(8,6);
% STEP2-1: Calculate Theta3.
% With "pf" on the axis of E4, E5, E6 and "pk" on the axis of E1, E2.
% We apply (noap*gs0^-1) to "pf" and take the norm of the diffence of that
% resulting point and "pk". Doing so we can calculate Theta3 applying the
% Canonic problem PADEN-KAHAN-THREE, because the screws E4,E5,E6 do not affect
% "pf" and the E1,E2 do not affect the norm of a vector with an end on "pk"
% resulting the problem ||exp(E3^theta3)*pf-pk||=||noap*gs0^-1*pf-pk||
% which by PADEN-KAHAN-THREE has none, one or two solutions for t31 t32.
noapHst0if = noapST24R*(Hst0\[pf; 1]); pkp = noapHst0if(1:3);
de = norm(pkp - pk);
t3 = PadenKahanThree(Twist(:,3), pf, pk, de);
Theta(1,3) = t3(1); Theta(5,3) = t3(2); % put results into Theta
Theta(3,:) = Theta(1,:); Theta(7,:) = Theta(5,:); % prepare Theta for next
%
% STEP2-2: Calculate Theta1 & Theta2.
% With "pf" on the axis of E4, E5, E6 we apply (noap*gs0^-1) to "pf" and
% the POE E1..E6 also to "pf" having already known the value for Theta3
% resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws
% E4,E5,E6 do not affect "pf" and the E3 is known (two values),resulting
% the problem exp(E1^theta1)*exp(E2^theta2)*pf' = noap*gs0^-1*pf
% which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions
% t11-t21 & t12-t22 for each value of t3, but we have two, then consider
% for t31 we get t11-t21 & t12-t22 & for t32 we get t13-t23 & t14-t24.
t1t2 = zeros(4,2);
for i = 1:2                   % for the TWO values of t3.
    j = i+fix(i/2);
    pfpt = expScrew([Twist(:,3);t3(i)])*[pf; 1];
    pfp = pfpt(1:3);
    t1t2(j:j+1,1:2) = PadenKahanTwo(Twist(:,1),Twist(:,2),pfp,pkp);
end
for i = 1:2:7
    j = i-fix(i/2);
    Theta(i,1:2) = t1t2(j,1:2); % put t1t2 values into Theta
    Theta(i+1,:) = Theta(i,:);  % prepare Theta for next step
end
%
% STEP2-3: Calculate Theta4 & Theta5.
% With "pp" on the axis of E6 apply E3^-1*E2^-1*E1^-1*noap*gs0^-1 to "pp"
% and also the POE E4*E5*E6 to "pp" knowing already Theta3-Theta2-Theta1,
% resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws
% E6 does not affect "pp" & Th3-Th2-Th1 known (four solutions), the problem
% exp(E4^theta4)*exp(E5^theta5)*pp = pk2p ; with
% pk2p = exp(E3^Th3)^-1*exp(E2^Th2)^-1*exp(E1^Th1)^-1*noap*gs0^-1*pp 
% which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions:
% t31,t21,t11 to t41-t51 & t42-t52 ; t31,t22,t12 to t43-t53 & t44-t54
% t32,t23,t13 to t45-t55 & t46-t56 ; t32,t24,t14 to t47-t57 & t48-t58
%
t4t5 = zeros(8,2);
noapHst0ip = noapST24R*(Hst0\[pp; 1]); 
for i = 1:2:7                     % for the 4 values of t3-t2-t1.
    pk2pt = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0ip;
    pk2pt = (expScrew([Twist(:,2);Theta(i,2)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,3);Theta(i,3)]))\pk2pt;
    pk2p = pk2pt(1:3);
    t4t5(i:i+1,1:2) = PadenKahanTwo(Twist(:,4),Twist(:,5),pp,pk2p); 
end
    Theta(:,4:5) = t4t5;          % put t4t5 values into Theta
%
% STEP2-4: Calculate Theta6.
% With "po" not in the axis of E6 apply E5^-1...*E1^-1*noap*gs0^-1 to "po"
% and applying E6 to "po" knowing already Theta5...Theta1 (8 solutions),
% resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
% exp(E6^theta6)*po = pk3p ; with
% pk3p = exp(E5^Th5)^-1*...*exp(E1^Th1)^-1*noap*gs0^-1*po 
% which by PADEN-KAHAN-ONE has none or one solution. Then for all
% Th5-Th4-Th3-Th2-Th1 known (eight solutions) we get t61...t68:
%
noapHst0io = noapST24R*(Hst0\[po; 1]);
for i = 1:size(Theta,1)
    pk2pt = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0io;
    pk2pt = (expScrew([Twist(:,2);Theta(i,2)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,3);Theta(i,3)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,4);Theta(i,4)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,5);Theta(i,5)]))\pk2pt;
    pk3p = pk2pt(1:3);
    Theta(i,6) = PadenKahanOne(Twist(:,6), po, pk3p);
end
ThetaST24R = Theta
%
tIKST24R = round(toc*1000,0);
time_IK_ST24R = ['Time to solve IK Screw Theory ', num2str(tIKST24R),' ms']
%
% STEP3: Test the solution applying ForwardKinemats to Robot
TwMag = [Twist; ThetaST24R(1,:)];
HstR = ForwardKinematicsPOE(TwMag);
TcpST24R = HstR * Hst0
%
%% Application "RST - Robotics System Toolbox - MATLAB" - for Kinematics.
%
% Mechanical characteristics of the PUMA Robot.
% ATTENTION! because for the RST standard the coordinate system of the base
% has the "Z" axis in the vertical direction (instead of the "Y" axis)
% Therefore the configuration of the robot has other coordinates.
%
tformj1 = rotX2tform(pi/2);
tformj2 = trvY2tform(pk(2))*rotX2tform(-pi/2);
tformj3 = rotY2tform(pi/2);
tformj4 = trvX2tform(pk(2)-pr(2));
tformj5 = rotY2tform(-pi/2);
tformj6 = trvZ2tform(pf(2)-pr(2))*rotY2tform(pi/2);
tformj7 = trvY2tform(-pp(3))*rotX2tform(pi/2)*rotZ2tform(pi/2);
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
body7 = robotics.RigidBody('body7');
jnt7 = robotics.Joint('jnt7','revolute');
setFixedTransform(jnt1,tformj1);
setFixedTransform(jnt2,tformj2);
setFixedTransform(jnt3,tformj3);
setFixedTransform(jnt4,tformj4);
setFixedTransform(jnt5,tformj5);
setFixedTransform(jnt6,tformj6);
setFixedTransform(jnt7,tformj7);
body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;
addBody(robot,body1,'base')
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
addBody(robot,body6,'body5')
addBody(robot,body7,'body6')
%
% STEP1: Apply ForwardKinemats to the Robot.
% Verify that your robot was built properly by using the |showdetails| or
% showdetails(robot);
% show(robot);
% getTransform permits to evaluate the Forward Kinematics.
joints = {'jnt1','jnt2','jnt3','jnt4','jnt5','jnt6','jnt7'};
MagRST = {0,Mag(1),Mag(2),Mag(3),Mag(4),Mag(5),Mag(6)};
FKRST = struct('JointName', joints,'JointPosition', MagRST);
noapRST = getTransform(robot,FKRST,'body7','body1')
%
% STEP2: Calculate the IK solutions with Robtics System Toolbox
tic;
noapRSTbase = getTransform(robot,FKRST,'body7','base');
weights = [1e-6,1e-6,1e-6,1e-6,1e-6,1e-6];
qinitial = robot.homeConfiguration;
IKRST = robotics.InverseKinematics('RigidBodyTree', robot);
IKRST.SolverParameters.MaxIterations= 15000;
[IKRSTsol,solinfo] = IKRST('body7',noapRSTbase,weights,qinitial);
ThetaRST7 = {IKRSTsol(1).JointPosition; IKRSTsol(2).JointPosition;
            IKRSTsol(3).JointPosition; IKRSTsol(4).JointPosition;
            IKRSTsol(5).JointPosition; IKRSTsol(6).JointPosition;
            IKRSTsol(7).JointPosition}';
ThetaRSTn = cell2mat(ThetaRST7);
ThetaRST = ThetaRSTn(2:7)
tIKRST = round(toc*1000,0);
time_IK_RST = ['Time to solve IK RS Toolbox ', num2str(tIKRST),' ms']
%
% STEP3: Test the solution applying ForwardKinemats to Robot
FKRST = struct('JointName', joints,'JointPosition', ThetaRST7);
TcpRST = getTransform(robot,FKRST,'body7','body1')
%