%% Screw Theory in Robotics
% An Illustrated and Practicable Introduction to Modern Mechanics
% by CRC Press
% © 2022 Jose M Pardos-Gotor
%
%% Ch3 - FORWARD KINEMATICS.
%
% Exercise 3.3.8: Redundant Robots (e.g., KUKA IIWA)
%
% Screw Theory POE.
% The goal of this exercise is to TEST
% Calculate the Homogeneous Matrix transformation of the end-effector for
% a Redundant robot KUKA IIWA with seven Joints.
%
% Mechanical characteristics of the Robot (AT REF HOME POSITION):
% po = Origen for he STATIONARY system of reference.
% pk = point in the crossing of the DOF Th1(rot) & Th2(rot) & Th3(rot).
% pr = point in the axis of Th4(rot) Th5(rot).
% pf = point in the crossing of the DOF Th5(rot), Th6(rot), Th7(rot).
% pp = TcP Tool Center Point
% hst0 = Tool (TcP) configuration (rot+tra) at robot reference position. 
%
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
%
Mag = zeros(1,7);
for i = 1:7
    Mag(i) = (rand-rand)*pi; % for testing various Theta1-Theta6
end
Mag
%
% Mechanical characteristics of the Robot:
po=[0;0;0]; pk=[0;0;0.36]; pr=[0;0;0.78];
pf=[0;0;1.18]; pp=[0.2;0;1.18];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pk po pr po pf pf];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ AxisY AxisZ -AxisY AxisZ -AxisY AxisX];
Twist = zeros(6,7);
for i = 1:7
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp)*rotY2tform(pi/2);
%
% STEP1: Apply ForwardKinemats to the Robot.
TwMag = [Twist; Mag]; % assign the rand values to joint Theta magnitudes.
HstR = ForwardKinematicsPOE(TwMag);
noap = HstR * Hst0
%