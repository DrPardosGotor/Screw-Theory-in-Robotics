%% "Fcn_ABBIRB120_ID_ToolD" Inverse Dynamics IRB120 ToolDown.
%
% Solves rhe INVERSE DYNAMICS for a desired  position, velocity 
% acceleration of the joints the ABB IRB120 Robot.
%
% function T12T6 = Fcn_ABBIRB120_ID_ToolD(u)
%
% The inputs "u" are composed by the following vectors.
% "Q12Q6" (6x1) desired POSITIONS for joints.
% "Qp12Qp6" (6x1) desired VELOCITIES for joints.
% "QpP12QpP6" (6x1) desired ACCELERATIONS for joints.
% "PoAcc" (3x1) vector for the potential of graviy (x;y;z) 
% The typical example is PoAcc = [0 0 -9.81] for gravity on -Z;
% The outputs:
% "T12T6" (6x1) necessary TORQUES for the joints.
%
% Mechanical characteristics of the PUMA Robot (AT REF POSITION):
% po = Origen for he STATIONARY system of reference.
% pk = point in the crossing of the DOF Th1(rot) & Th2(rot).
% pr = point in the axis of Th3(rot).
% pf = point in the crossing of the DOF Th4(rot), Th5(rot), Th6(rot).
% pp = TcP Tool Center Point
% hst0 = Tool (TcP) configuration (rot+tra) at robot reference position.
%
% Next code gets the Twist & Hst0 for the IRB120 robot:
% po=[0;0;0]; pk=[0;0.290;0]; pr=[0;0.560;0];
% pf=[0.302;0.630;0]; pp=[0.302;0.470;0];
% AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
% Axis = [AxisY AxisZ AxisZ AxisX AxisZ -AxisY]; % Attention to Axis6 (-)
% Point = [pk pk pr pf pf pp];
% Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
% Twist = zeros(6,6);
% for i = 1:6
%    Twist(:,i) = Joint2Twist(Axis(:,i), Point(:,i), Joint(i,:));
% end
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
% You should have received a copy of the GNU Leser General Public License
% along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2019/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% Fcn_ABBIRB120_ID_ToolD
%
function T12T6 = Fcn_ABBIRB120_ID_ToolD(u) %#codegen
%
% Mechanical characteristics of the IRB120 Robot:
Twist = [           0    0.2900    0.5600         0    0.6300         0;
                    0         0         0         0   -0.3020         0;
                    0         0         0   -0.6300         0   -0.3020;
                    0         0         0    1.0000         0         0;
               1.0000         0         0         0         0   -1.0000;
                    0    1.0000    1.0000         0    1.0000         0];
%Hst0 = [0 -1 0 0.302; 0 0 -1 0.470;1 0 0 0;0 0 0 1];
%
CM1 = [0; 0.25; 0]; CM2 = [0; 0.4; 0]; CM3 = [0.15; 0.63; 0];
CM4 = [0.2; 0.63; 0]; CM5 = [0.3; 0.63; 1]; CM6 = [0.3; 0.5; 0];
IT1 = [0.1; 0.2; 0.3]; IT2 = [0.3; 0.1; 0.5]; IT3 = [0.1; 0.1; 0.1];
IT4 = [0.1; 0.2; 0.3]; IT5 = [0.3; 0.1; 0.5]; IT6 = [0.1; 0.1; 0.1];
mass = [7 6 5 4 3 2];
LiMas = [CM1 CM2 CM3 CM4 CM5 CM6;IT1 IT2 IT3 IT4 IT5 IT6; mass];
%
%
Th = u(1:6);
Thp = u(7:12);
Thpp = u(13:18);
PoAcc = u(19:21)';
%
TwMag = [Twist; Th];
%
% We solve it with the ST24R "Screw Theory Toolbox for Robotics".
% complete algebraic solution.
%
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
T12T6 = MtST24RJsl*Thpp' + CtdtST24RAij*Thp' + NtST24RWre;
%
end
%
%