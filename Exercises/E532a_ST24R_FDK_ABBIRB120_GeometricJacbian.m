%% Screw Theory - FORWARD DIFFERENTIAL Kinematics
% ABB IRB 120.
% GEOMETRIC Jacobian.
%
% The goal of this exercise is to prove the FORWARD (Tcp Velocities)
% differential kinematics, based on the known or desired Joint Velocities.
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
%% E532a_ST24R_FDK_ABBIRB120_GeometricJacbian
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
% The APPLIED JOINT VELOCITIES.
Thetap = [-0.0073 -0.8807 1.6060 0.0000 0.7253 0.9127]';
%
%%%%%%%%%%%%%%%%%%%%%%
% TRAJECTORY TARGET
% for the exercise of the Screw Theory for Robotics Handbook.
% The point t1 of the TcP TRAJECTORY is defined by the "u" (1x6) vector
% which constains Traslation X+Y+Z and Rotation X+Y+Z.
t1goal = [-0.25 0.125 0.4 pi/2 0 pi]
% 
%%%%%%%%%%%%%%%%%%%%%%
% INVERSE KINEMATICS to get a set of solutions for joint magnitudes
ThetaSet = Fcn_ST24R_IK_ABBIRB120_ToolD_YUp(t1goal);
% From the set of solutions we choose only one to proceed.
Theta = ThetaSet(3,:)
%
%%%%%%%%%%%%%%%%%%%%%%
% Apply Forward Kinematics to check wether the point in trajectory is 
% in the workspace of the robot.
TwMag = [Twist; Theta];
noap = ForwardKinematicsPOE(TwMag) * Hst0;
t1real = [noap(1:3,4)' tform2eul(noap,'XYZ')]
%
%%%%%%%%%%%%%%%%%%%%%%
% Solve the FORWARD DIFFERENTIAL KINEMATICS.
% Tool SPATIAL VELOCITY in terms of the screw theory VstS, with the
% The Direct GEOMETRIC JACOBIAN by DEFINITION.
VstS = GeoJacobianS(TwMag) * Thetap;
% Tool carthesian Velocity in the Spatial frame VtS.
VtS = [VstS(1:3)+axis2skew(VstS(4:6))*t1real(1:3)'; VstS(4:6)]
%