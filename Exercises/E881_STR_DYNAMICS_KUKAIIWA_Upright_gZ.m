%% Screw Theory - DYNAMICS - KUKA IIWA R820 - Home Upright position
% & Gravity acting in direction -Z (gZ).
% REDUNDANT & COLLABORATIVE robot.
%
% The goal of this exercise is to prove the DYNAMICS
% M(t)*ddt + C(t,dt)*dt + N(t) = T
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
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
% Copyright (C) 2003-2019, by Dr. Jose M. Pardos-Gotor.
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
% Revision 1.1  2019/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% E881_STR_DYNAMICS_KUKAIIWA_Upright_gZ
%
clear
clc
%
n = 7;
% Theta Value, the current joint magnitudes.
ThVal = zeros(1,n);
% Theta Desired, the necessary joint magnitudes to comply with the task.
ThDes = ThVal;
for i = 1:n
    ThVal(i) = (rand-rand)*pi;
    ThDes(i) = (rand-rand)*pi;
end
%
% Robot Technical Data.
% Maximum RANGE for the robot joints rad +/-, (by catalog).
Thmax = pi/180*[170 120 170 120 170 120 175];
% Maximum SPEED for the robot joints rad/sec, (by catalog).
Thpmax = pi/180*[85 85 100 75 130 135 135];
% Maximum TORQUE for the robot joints Nm, (by catalog).
Tdynmax = [320 320 176 176 110 40 40];
%
% Trajectory Desired for any joint with quadratic form:
% Th = a/2*t^2 => Thp = a*t => Thpp = a
% ttall is the maximum of minimum time for any joint trajectory at Thpmax.
% ti = 2*Thi/Thpmaxi ; ttall = max(ti)
ttall = max(2*(Thpmax.^-1)*diag(abs(ThDes-ThVal)));
% Choose this ttall time for all joint to make ISOCHRONOUS trajectory. So,
% ThpDes are the VELOCITIES desired for rhe joints in rad/sec.
% ThpDes = a*ttall = 2*Thi/
% ThppDes ACCELERATIONS desired for the robot joints rad/sec^2.
% ThppDes = 2*Thi/ttall^2
if ttall == 0
    ThpDes = zeros(1,n);
    ThppDes = zeros(1,n);
else
    ThpDes = 2*(ThDes-ThVal)/ttall;
    ThppDes = ThpDes/ttall;   
end
%
%
% Mechanical characteristics of the Robot (AT REF HOME POSITION):
% Gravity definition:
% The S Spatial system has the "Z" axis up (i.e., -g direction).
PoAcc = [0 0 -9.80665]';
% LiMas matrix stands for "Link Mass" and is a 7x7 matrix.
LiMas = [0         0         0         0         0         0         0;
   -0.0300    0.0420    0.0300   -0.0340   -0.0210    0.0004         0;
    0.2775    0.4190    0.6945    0.8470    1.0410    1.1810    1.2810;
    0.1000    0.0180    0.0800    0.0300    0.0200    0.0050    0.0010;
    0.0900    0.0500    0.0750    0.0290    0.0180    0.0036    0.0010;
    0.0200    0.0440    0.0100    0.0100    0.0050    0.0047    0.0010;
    4.0000    4.0000    3.0000    2.7000    1.7000    1.8000    0.3000];
%
% Joints TWISTS definition:
% Axes definition
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]';
po=[0;0;0]; pk=[0;0;0.36]; pr=[0;0;0.78]; pf=[0;0;1.18]; pp=[0;0;1.18];
Point = [po pk po pr po pf po];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ AxisY AxisZ -AxisY AxisZ AxisY AxisZ];
Twist = zeros(6,n);
for i = 1:n
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
TwMag = [Twist; ThDes];
%
%
%
% FIRST we solve it with the ST24R "Screw Theory Toolbox for Robotics".
% symbolic solution for and N(t).
%
tic;
% M(t) Inertia matrix by the use of Jsl LINK Jacobian.
MtST24RJsl = MInertiaJsl(TwMag,LiMas);
%
% C(t,dt) Coriolis matrix by the use of Aij Adjoint transformation.
CtdtST24RAij = CCoriolisAij(TwMag,LiMas,ThpDes);
%
% N(t) Potential Matrix by the use of Symbolic differentiation.
NtST24RSym = NPotentialDifSym(TwMag,LiMas,PoAcc);
%
% Inverse Dynamics solution for the joint TORQUES T.
TdynST24RSym = MtST24RJsl*ThppDes' + CtdtST24RAij*ThpDes' + NtST24RSym
toc
%
%
% SECOND we solve it with the ST24R "Screw Theory Toolbox for Robotics".
% complete algebraic solution with Aij Adjoint transformation functions.
%
tic;
% M(t) Inertia matrix by the use of Aij Adjoint transformation.
MtST24RAij = MInertiaAij(TwMag,LiMas);
%
% C(t,dt) Coriolis matrix by the use of Aij Adjoint transformation.
CtdtST24RAij = CCoriolisAij(TwMag,LiMas,ThpDes);
%
% N(t) NEW Potential Matrix by the use of Aij Adjoint transformation.
NtST24RAij = NPotentialAij(TwMag,LiMas,PoAcc);
%
% Inverse Dynamics solution for the joint TORQUES T.
TdynST24RAij = MtST24RAij*ThppDes' + CtdtST24RAij*ThpDes' + NtST24RAij
toc
%
%
% THIRD we solve it with the ST24R "Screw Theory Toolbox for Robotics".
% using the Jacobian approach for Mt and Nt.
%
tic;
% M(t) Inertia matrix by the use of Jsl LINK Jacobian.
MtST24RJsl = MInertiaJsl_mex(TwMag,LiMas);
%
% C(t,dt) Coriolis matrix by the use of Aij Adjoint transformation.
CtdtST24RAij = CCoriolisAij_mex(TwMag,LiMas,ThpDes);
%
% N(t) NEW Potential Matrix by the use of the Spatial Jacobian.
NtST24RWre = NPotentialWre_mex(TwMag,LiMas,PoAcc);
%
% Inverse Dynamics solution for the joint TORQUES T.
TdynST24RWre = MtST24RJsl*ThppDes' + CtdtST24RAij*ThpDes' + NtST24RWre
toc
%
