%% Screw Theory - DYNAMICS - ABB IRB 120 - Home position Elbow & Tool down
% & Gravity acting in direction -Y (gY).
%
% The goal of this exercise is to prove the DYNAMICS
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
% M(t)*ddt + C(t,dt)*dt + N(t,dt) = T
%
% Mechanical characteristics of the Robot (AT REF HOME POSITION):
% po = Origen for he STATIONARY system of reference.
% pk = point in the crossing of the DOF Th1(rot) & Th2(rot).
% pr = point in the axis of Th3(rot).
% pf = point in the crossing of the DOF Th4(rot), Th5(rot), Th6(rot).
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
%% E840_STR_DYNAMICS_ABBIRB120_gY
%
clear
clc
%
n = 6;
Th = zeros(1,n); Thp = zeros(1,n); Thpp = zeros(1,n);
for i = 1:n
    Th(i) = (rand-rand)*pi; % for testing various Theta POS
    Thp(i) = (rand-rand)*pi; % for testing various Theta VEL
    Thpp(i) = (rand-rand)*pi; % for testing various Theta ACC
end
%
% Mechanical characteristics of the Robot (AT REF HOME POSITION):
% The S Spatial system has the "Y" axis up (i.e., -g direction).
%
PoAcc = [0 -9.81 0]';
CM1 = [0; 0.25; 0]; CM2 = [0; 0.4; 0]; CM3 = [0.15; 0.63; 0];
CM4 = [0.2; 0.63; 0]; CM5 = [0.3; 0.63; 1]; CM6 = [0.3; 0.5; 0];
IT1 = [0.1; 0.2; 0.3]; IT2 = [0.3; 0.1; 0.5]; IT3 = [0.1; 0.1; 0.1];
IT4 = [0.1; 0.2; 0.3]; IT5 = [0.3; 0.1; 0.5]; IT6 = [0.1; 0.1; 0.1];
mass = [7 6 5 4 3 2];
LiMas = [CM1 CM2 CM3 CM4 CM5 CM6;IT1 IT2 IT3 IT4 IT5 IT6; mass];
%
po=[0;0;0]; pk=[0;0.290;0]; pr=[0;0.560;0];
pf=[0.302;0.630;0]; pp=[0.302;0.470;0];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [pk pk pr pf pf pp];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisY AxisZ AxisZ AxisX AxisZ -AxisY];
Twist = zeros(6,6);
for i = 1:6
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvX2tform(pp(1))*trvY2tform(pp(2))*trvZ2tform(pp(3));
Hst0 = Hst0*rotX2tform(pi/2)*rotY2tform(0)*rotZ2tform(pi/2);
TwMag = [Twist; Th];
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
CtdtST24RAij = CCoriolisAij(TwMag,LiMas,Thp);
%
% N(t) Potential Matrix by the use of Symbolic differentiation.
NtST24RSym = NPotentialDifSym(TwMag,LiMas,PoAcc);
%
% Inverse Dynamics solution for the joint TORQUES T.
TdynST24RSym = MtST24RJsl*Thpp' + CtdtST24RAij*Thp' + NtST24RSym
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
CtdtST24RAij = CCoriolisAij(TwMag,LiMas,Thp);
%
% N(t) NEW Potential Matrix by the use of Aij Adjoint transformation.
NtST24RAij = NPotentialAij(TwMag,LiMas,PoAcc);
%
% Inverse Dynamics solution for the joint TORQUES T.
TdynST24RAij = MtST24RAij*Thpp' + CtdtST24RAij*Thp' + NtST24RAij
toc
%
%
% THIRD we solve it with the ST24R "Screw Theory Toolbox for Robotics".
% using the Jacobian approach for Mt and Nt.
%
tic;
% M(t) Inertia matrix by the use of Jsl LINK Jacobian.
MtST24RJsl = MInertiaJsl(TwMag,LiMas);
%
% C(t,dt) Coriolis matrix by the use of Aij Adjoint transformation.
CtdtST24RAij = CCoriolisAij(TwMag,LiMas,Thp);
%
% N(t) NEW Potential Matrix by the use of the Spatial Jacobian.
NtST24RWre = NPotentialWre(TwMag,LiMas,PoAcc);
%
% Inverse Dynamics solution for the joint TORQUES T.
TdynST24RWre = MtST24RJsl*Thpp' + CtdtST24RAij*Thp' + NtST24RWre
toc
%
% FORTH we prove the Fcn_ABBIRB120_ID_ToolD
%
tic;
TdynST24RFcn = Fcn_ABBIRB120_ID_ToolD_mex([Th Thp Thpp PoAcc'])
toc
%