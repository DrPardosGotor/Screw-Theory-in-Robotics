%% Screw Theory - INVERSE & FORWARD DIFFERENTIAL Kinematics
% KUKA IIWA14.
% GEOMETRIC Jacobian.
%
% For the exercise of the Screw Theory for Robotics Handbook, the aim is to
% demonstrate the differential kinematics formulations (inv + for).
% FIRST goal is to calculate the INVERSE (Joint Thetap Velocities)
% Diferential Kinematics, based on the known or desired Tool Velocities.
% SECOND goal is to calculate the FORWARD (Tool Velocities)
% Diferential Kinematics, based on the known Joint Velocities.
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
%% E536a_ST24R_FIDK_KUKAIIWA14_GeometricJacbian
%
clear;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mechanical characteristics of the Robot (AT REF HOME POSITION):
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The S Spatial system has the "Z" axis up (i.e., -g direction).
%
po=[0;0;0]; pk=[0;0;0.36]; pr=[0;0;0.78];
pf=[0;0;1.18]; pp=[0;0;1.38];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pk po pr po pf po];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ AxisY AxisZ -AxisY AxisZ AxisY AxisZ];
Twist = zeros(6,7);
for i = 1:7
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp);
%
% Maximum Magnitue for the robot joints POSITION rad, (by catalog).
% Thmax = pi/180*[170 120 170 120 170 120 175];
% Thmin = -pi/180*[170 120 170 120 170 120 175];
% Maximum SPEED for the robot joints rad/sec, (by catalog).
% Thpmax = pi/180*[85 85 100 75 130 135 135];
%
%%%%%%%%%%%%%%%%%%%%%%
% FORWARD KINEMATICS to get a target inside the robot workspace
Theta = [170*(rand-rand) 120*(rand-rand) 170*(rand-rand)];
Theta = [Theta 120*(rand-rand) 170*(rand-rand) 120*(rand-rand)];
Theta = [Theta 175*(rand-rand)];
Theta = Theta*pi/180;
TwMag = [Twist; Theta];
noap = ForwardKinematicsPOE(TwMag) * Hst0;
t1goal = [noap(1:3,4)' tform2eul(noap,'XYZ')]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The DESIRED TOOL (Carthesian) VELOCITIES for trajectory targets.
% VtS = the desired velocity for Tool Pose in spatial frame (S).
VtSr = [rand rand rand rand rand rand]'
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve the INVERSE DIFFERENTIAL KINEMATICS.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The Velocity for the Joints of the Robot with inverse GEOMETRIC JACOBIAN.
JstS = GeoJacobianS(TwMag);
Thetap = JstS\[VtSr(1:3)-axis2skew(VtSr(4:6))*t1goal(1:3)'; VtSr(4:6)]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve the FORWARD DIFFERENTIAL KINEMATICS.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tool SPATIAL VELOCITY in terms of the screw theory VstS, with the
% The Manipulator GEOMETRIC JACOBIAN by DEFINITION.
VstS = JstS * Thetap;
% Tool carthesian Velocity in the Spatial frame VtS.
VtS = [VstS(1:3)+axis2skew(VstS(4:6))*t1goal(1:3)'; VstS(4:6)]
%