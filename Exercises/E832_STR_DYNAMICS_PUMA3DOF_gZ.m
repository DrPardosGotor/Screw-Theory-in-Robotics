%% Screw Theory - DYNAMICS - Idealized PUMA 3 DOF "Murray exercise 4.3".
%
% The goal of this exercise is to prove the DYNAMICS
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% M(t)*ddt + C(t,dt)*dt + N(t,dt) = T
% with T generalized joint torques
% with t generalized joint positions, dt velocities and ddt accelerations.
%
% Attention because this exercise consideres a PUMA robot with only the
% first 3 DOF. It is a kind of three-link manipulator to exercise.
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
% http://www.preh
%
% CHANGES:
% Revision 1.1  2019/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%
%% E832_STR_DYNAMICS_PUMA3DOF_gZ
clear
clc
%
n = 3;
Th = zeros(1,n); Thp = zeros(1,n); Thpp = zeros(1,n);
for i = 1:n
    %Th(i) = (rand-rand)*pi; % for testing various Theta POS
    Th(1) = pi/4;
    Thp(i) = (rand-rand)*pi; % for testing various Theta VEL
    Thpp(i) = (rand-rand)*pi; % for testing various Theta ACC
end
%
% Mechanical characteristics of the Robot:
PoAcc = [0 0 -10]';
l0 = 1; l1 = 1; l2 = 1;
CM1 = [0; 0; 0.24]; CM2 = [0; l1/2; l0]; CM3 = [0; l1+l2/2; l0];
IT1 = [0.3; 0.3; 0.1]; IT2 = [0.5; 0.1; 0.5]; IT3 = [0.7; 0.2; 0.7];
mass = [1 1 1];
LiMas = [CM1 CM2 CM3;IT1 IT2 IT3; mass];
%
po=[0;0;0]; pk=[0;0;l0]; pr=[0;l1;l0]; 
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pk pr];
Joint = ['rot'; 'rot'; 'rot'];
Axis = [AxisZ -AxisX -AxisX];
%
Twist = zeros(6,n);
for i = 1:n
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
TwMag = [Twist; Th];
%
% We solve the exercise as showed by Murray:
%
% M(t) Inertia Matrix.
MtM11 = IT2(2)*sin(Th(2))^2+IT3(2)*sin(Th(2)+Th(3))^2+IT1(3);
MtM11 = MtM11 + IT2(3)*cos(Th(2))^2+IT3(3)*cos(Th(2)+Th(3))^2;
MtM11 = MtM11 + mass(2)*(l1/2)^2*cos(Th(2))^2;
MtM11 = MtM11 + mass(3)*(l1*cos(Th(2))+l2/2*cos(Th(2)+Th(3)))^2;
MtM22 = IT2(1)+IT3(1)+mass(3)*l1^2+mass(2)*(l1/2)^2+mass(3)*(l2/2)^2;
MtM22 = MtM22 + 2*mass(3)*l1*l2/2*cos(Th(3));
MtM23 = IT3(1)+mass(3)*(l2/2)^2+mass(3)*l1*l2/2*cos(Th(3));
MtM32 = MtM23;
MID33 = IT3(1)+mass(3)*(l2/2)^2;
MtM =[MtM11 0 0; 0 MtM22 MtM23; 0 MtM32 MID33];
%
% C(t,dt) Coriolis Matrix by Murray
T112 = (IT2(2)-IT2(3)-mass(2)*(l1/2)^2)*cos(Th(2))*sin(Th(2));
T112 = T112 + (IT3(2)-IT3(3))*cos(Th(2)+Th(3))*sin(Th(2)+Th(3));
T112 = T112 - mass(3)*(l1*cos(Th(2))+l2/2*cos(Th(2)+Th(3)))*(l1*sin(Th(2))+l2/2*sin(Th(2)+Th(3)));
T113 = (IT3(2)-IT3(3))*cos(Th(2)+Th(3))*sin(Th(2)+Th(3));
T113 = T113 - mass(3)*l2/2*sin(Th(2)+Th(3))*(l1*cos(Th(2))+l2/2*cos(Th(2)+Th(3)));
T121 = T112;
T131 = T113;
T211 = (IT2(3)-IT2(2)+mass(2)*(l1/2)^2)*cos(Th(2))*sin(Th(2));
T211 = T211 + (IT3(3)-IT3(2))*cos(Th(2)+Th(3))*sin(Th(2)+Th(3));
T211 = T211 + mass(3)*(l1*cos(Th(2))+l2/2*cos(Th(2)+Th(3)))*(l1*sin(Th(2))+l2/2*sin(Th(2)+Th(3)));
T223 = -l1*mass(3)*l2/2*sin(Th(3));
T232 = T223;
T233 = T223;
T311 = (IT3(3)-IT3(2))*cos(Th(2)+Th(3))*sin(Th(2)+Th(3));
T311 = T311 + mass(3)*l2/2*sin(Th(2)+Th(3))*(l1*cos(Th(2))+l2/2*cos(Th(2)+Th(3)));
T322 = -T223;
CtdtM11 = T112*Thp(2)+T113*Thp(3);
CtdtM12 = T121*Thp(1);
CtdtM13 = T131*Thp(1);
CtdtM21 = T211*Thp(1);
CtdtM22 = T223*Thp(3);
CtdtM23 = T232*Thp(2)+T233*Thp(3);
CtdtM31 = T311*Thp(1);
CtdtM32 = T322*Thp(2);
CtdtM = [CtdtM11 CtdtM12 CtdtM13; CtdtM21 CtdtM22 CtdtM23; CtdtM31 CtdtM32 0];
%
% N(t) Potential Matrix by Murray
Nt21 = PoAcc(3)*(mass(2)*l1/2+mass(3)*l1)*cos(Th(2));
Nt21 = Nt21 + PoAcc(3)*mass(3)*l2/2*cos(Th(2)+Th(3));
Nt31 = PoAcc(3)*mass(3)*l2/2*cos(Th(2)+Th(3));
NtM = [0; Nt21; Nt31];
%
% The inverse Dynamics solution (even though is not explicit in Murray's).
TdynMurray = MtM*Thpp' + CtdtM*Thp' + NtM
%
%
%
% To follow we solve the DYNAMICS in three different ways using the 
% Screw Theory and Lie Algebra tools.
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