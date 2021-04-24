%% Screw Theory - INVERSE DIFFERENTIAL Kinematics.
% ABB IRB 910SC
%
% The goal of this exercise is to get the inverse (Joint Velocities) for a 
% given tool velocities, based on the use of the GEOMETRIC JACOBIAN
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
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
% http://www.preh
%
% CHANGES:
% Revision 1.1  2020/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% E535b_ST24R_IDK_ABBIRB910_GeometricJacobian.m
%
clear
clc
%
% the magnitudes for the Joints.
Theta = [-pi/2 -pi/2 0 0; -pi/2 -2.7 0 0; -pi/2 -3.141 0 0; pi/2 -2.7 0 0];
% the velocity for TcP Position.
TcPp = [0.14 0 -0.23 0 -0.92 0]'
%
ThetapGJ = zeros(size(Theta,2),size(Theta,1));
%
% Mechanical characteristics of the Robot:
l1 = 0.4; l2 = 0.25; l3 = 0.125;
po=[0;0;0]; pr=[l1;0;0]; pf=[l1+l2;0;0]; pp=[l1+l2;l3;0]; 
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pr pf pp];
Joint = ['rot'; 'rot'; 'tra'; 'rot'];
Axis = [AxisY AxisY AxisY -AxisY];
Twist = zeros(6,4);
for i = 1:4
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp)*rotX2tform(pi/2)*rotZ2tform(-pi);
%
for i = 1:size(Theta)
tic;
% Apply ForwardKinemats to the Robot.
TwMag = [Twist; Theta(i,:)]; 
HstR = ForwardKinematicsPOE(TwMag);
noap = HstR * Hst0;
%
% The MANIPULATOR JACOBIAN by DEFINITION.
JstS = GeoJacobianS(TwMag);
%
% The Velocity for the Joints of the Robot is:
%ThetapGJ = JstS\[TcPp(1:3)-cross(TcPp(4:6),noap(1:3,4)); TcPp(4:6)]
ThetapGJ(:,i) = JstS\[TcPp(1:3)-axis2skew(TcPp(4:6))*noap(1:3,4); TcPp(4:6)]
tIGJ = round(toc*1000,1);
time_IK_GJ = ['Time differential inverse kinematics ', num2str(tIGJ),' ms']
%
end
%