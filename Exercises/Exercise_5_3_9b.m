%% Screw Theory in Robotics
% An Illustrated and Practicable Introduction to Modern Mechanics
% by CRC Press
% © 2022 Jose M Pardos-Gotor
%
%% Ch5 - DIFFERENTIAL KINEMATICS.
%
% Exercise 5.3.9b: ABB IRB 120 - INVERSE Geometric Jacobian.
%
% The goal of this exercise is to solve for the Joint Velocities, taken
% the Tool pose velocities (i.e. TcP position and T frame rotation vel).
% The calculation is based on: Inverse GEOMETRIC JACOBIAN
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% FIST
% define the INPUTS are:
% VtS = the desired velocity for Tool Pose in spatial frame (S).
% Path = Trajectory in the plane X-Z defined by four points t1..t4.
% SECOND 
% For each point in the trajectory we need the joints magnitudes, Therefore
% we must solve the Inverse Kinematics problem for the robot in the poses.
% THIRD
% Calculate the TWISTS for the robot.
% Calculate the GEOMETRIC JACOBIAN.
% Solve the joint velocities for the trajectory with the inverse Jacobian
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
% http://www.preh
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mechanical characteristics of the Robot (AT REF HOME POSITION):
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Maximum SPEED for the robot joints rad/sec, (by catalog).
% Thpmax = pi/180*[250 250 250 320 320 420];
%
po=[0;0;0]; pk=[0;0.290;0]; pr=[0;0.560;0];
pf=[0.302;0.630;0]; pp=[0.302;0.470;0];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [pk pk pr pf pf pp];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisY -AxisZ -AxisZ AxisX -AxisZ -AxisY];
Twist = zeros(6,6);
for i = 1:6
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp)*rotX2tform(pi/2)*rotZ2tform(pi);
%
%%%%%%%%%%%%%%%%%%%%%%
% The DESIRED TOOL (Carthesian) VELOCITIES for all trajectory targets.
% VtS = the desired velocity for Tool Pose in spatial frame (S).
VtS = [0.1402 0 -0.2308 0 -0.92 0]'
%
% Path = Trajectory in the plane X-Z defined by four points t1..t4.
% t1..4 (6x1) = point in the trajectory defining for the Tool Pose the
% "Tpos" (3x1) position and "Trot" (3x1) rotation (order X+Y+Z).
t1 = [-0.25 0.125 0.4 pi/2 0 pi];
t2 = [-0.1 0.125 0.17 pi/2 0 pi];
t3 = [-0.0001 0.125 0.15 pi/2 0 pi];
t4 = [0.1 0.125 -0.17 pi/2 0 pi];
Path = [t1; t2; t3; t4];
%
% For each point in the trajectory we need the joints magnitudes, as the 
% Geometric Jacobian is a valued matrix. Therefore we must solve the 
% Inverse Kinematics problem for the robot in the t1..t4 poses.
% We know that in general there are 8 solutions for each robot pose, then
% we select only one valid solution (for instance the #5).
% Theta = matrix where to save all sets of joint magnitudes for each pose.
Theta = zeros(size(Path,1),6);
for i = 1:size(Theta)
    Th = Fcn_ST24R_IK_ABBIRB120_ToolD_YUp(Path(i,:));
    Theta(i,:) = Th(3,:);
end
Theta
%
%%%%%%%%%%%%%%%%%%%%%%
% JOINT VELOCITIES (Output)
% ThetapGJ (6xn)= matrix with the results, these are the values for the
% joints velocities for each point in the trajectory of the Tool. Results
% are stored by columns.
ThetapGJ = zeros(size(Theta,2),size(Theta,1));
%
tic;
for i = 1:size(Theta)
% The MANIPULATOR JACOBIAN by DEFINITION.
TwMag = [Twist; Theta(i,:)];
JstS = GeoJacobianS(TwMag);
%
% The Velocity for the Joints of the Robot with inverse GEOMETRIC JACOBIAN.
ThetapGJ(:,i) = JstS\[VtS(1:3)-axis2skew(VtS(4:6))*Path(i,1:3)'; VtS(4:6)];
%
end
tIGJ = round(toc*1000,1);
time_IK_GJ = ['Time differential inverse kinematics ', num2str(tIGJ),' ms']
ThetapGJ
%