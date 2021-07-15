%% Screw Theory in Robotics
% An Illustrated and Practicable Introduction to Modern Mechanics
% by CRC Press
% © 2022 Jose M Pardos-Gotor
%
%% Ch3 - FORWARD KINEMATICS.
%
% Exercise 3.3.7: Collaborative Robots (e.g., UNIVERSAL UR16e)
%
% Screw Theory POE.
% Calculate the Homogeneous Matrix transformation for the end-effector of
% a UNIVERSAL UR16e collaborartive type robot of six Joints.
%
% Using Screw Theory Functions from ST24R.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
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
% n is number of DOF.
n = 6;
Mag = zeros(1,n);
for i = 1:6
    Mag(i) = (rand-rand)*pi; % for testing various Theta1-Theta6
end
% Mechanical characteristics of the IRB120 Robot:
% Denavit-Hartenberg parameters for the UR Arms:
d1 = 0.181; a2 = 0.478; a3 = 0.360; d4 = 0.174; d5 = 0.120; d6 = 0.190;
%
po=[0; 0; 0]; pk=[0; 0; d1]; pr=[a2; 0; d1]; pf=[a2+a3; d4; d1];
pg=[a2+a3; d4; d1-d5]; pp=[a2+a3; d4+d6; d1-d5];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pk pr pf pg pp];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ AxisY AxisY AxisY -AxisZ AxisY];
Twist = zeros(6,6);
for i = 1:6
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp)*rotX2tform(-pi/2)*rotZ2tform(pi);
%
TwMag = [Twist; Mag]; % assign the rand values to joint Theta magnitudes.
noap = ForwardKinematicsPOE(TwMag) * Hst0
%