%% Screw Theory - FORWARD & INVERSE DIFFERENTIAL Kinematics
% ABB IRB120.
% GEOMETRIC Jacobian.
%
% For the exercise of the Screw Theory for Robotics Handbook, the aim is to
% demonstrate the differential kinematics formulations (inv + for).
% FIRST goal is to calculate the INVERSE (Joint Thetap Velocities) for
% Diferential Kinematics, based on the known or desired Tool Velocities.
% SECOND goal is to calculate the FORWARD (Tool Velocities) for
% Diferential Kinematics, based on the known or desired Joint Velocities.
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
%% E532c_ST24R_FIDK_ABBIRB120_GeometricJacbian
%
clear;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mechanical characteristics of the Robot (AT REF HOME POSITION):
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
% Motion RANGE for the robot joints POSITION rad, (by catalog).
% Thmax = pi/180*[165 110 70 160 120 400];
% Thmin = -pi/180*[165 110 110 160 120 400];
% Maximum SPEED for the robot joints rad/sec, (by catalog).
% Thpmax = pi/180*[250 250 250 320 320 420];
%
%%%%%%%%%%%%%%%%%%%%%%
% The DESIRED TOOL (Carthesian) VELOCITIES for all trajectory targets.
% VtS = the desired velocity for Tool Pose in spatial frame (S).
VtS = [rand rand rand rand rand rand]'
%
%%%%%%%%%%%%%%%%%%%%%%
% FORWARD KINEMATICS to get a target inside the robot workspace
Theta = [165*(rand-rand) 110*(rand-rand) 70*(rand-rand)];
Theta = [Theta 160*(rand-rand) 120*(rand-rand) 400*(rand-rand)];
Theta = Theta*pi/180;
TwMag = [Twist; Theta];
noap = ForwardKinematicsPOE(TwMag) * Hst0;
t1goal = [noap(1:3,4)' tform2eul(noap,'XYZ')]
%
%%%%%%%%%%%%%%%%%%%%%%
% INVERSE KINEMATICS
% Solve Inverse Kinematics to get a set of possible solutions for joint
ThetaSet = Fcn_ST24R_IK_ABBIRB120_ToolD_YUp(t1goal);
% From the set of solutions we choose only one to proceed.
Theta = ThetaSet(3,:)
%
% Apply Forward Kinematics to check wether the point in trajectory is 
% in the workspace of the robot.
TwMag = [Twist; Theta];
noap = ForwardKinematicsPOE(TwMag) * Hst0;
t1real = [noap(1:3,4)' tform2eul(noap,'XYZ')]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve the INVERSE DIFFERENTIAL KINEMATICS.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The Velocity for the Joints of the Robot with inverse GEOMETRIC JACOBIAN.
JstS = GeoJacobianS(TwMag);
Thetap = JstS\[VtS(1:3)-axis2skew(VtS(4:6))*t1real(1:3)'; VtS(4:6)]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve the FORWARD DIFFERENTIAL KINEMATICS.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tool SPATIAL VELOCITY in terms of the screw theory VstS, with the
% The Manipulator GEOMETRIC JACOBIAN by DEFINITION.
VstS = JstS * Thetap;
% Tool carthesian Velocity in the Spatial frame VtS.
VtS = [VstS(1:3)+axis2skew(VstS(4:6))*t1real(1:3)'; VstS(4:6)]
%