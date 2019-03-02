%% Screw Theory - DYNAMICS - Idealized SCARA "Murray exercise 4.4".
%
% The goal of this exercise is to prove the DYNAMICS
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% M(t)*ddt + C(t,dt)*dt + N(t,dt) = T
% with T generalized joint torques
% with t generalized joint positions, dt velocities and ddt accelerations.
%
% Attention because this exercise consideres a SCARA robot with translation
% joint in the last (fourth) DoF t4.
% It is different with other SCARA robots, like the ABB IRB910SC, which has
% the translation on the third DoF t3.
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
%% E872_STR_DYNAMICS_SCARA
clear
clc
%
n = 4;
Th = zeros(1,n); Thp = zeros(1,n); Thpp = zeros(1,n);
for i = 1:n
    Th(i) = (rand-rand)*pi; % for testing various Theta POS
    Thp(i) = (rand-rand)*pi; % for testing various Theta VEL
    Thpp(i) = (rand-rand)*pi; % for testing various Theta ACC
end
%
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
% Mechanical characteristics of the Robot:
l1 = 0.4; l2 = 0.25; l0 = 0.125;
%
% Joints TWISTS definition:
po=[0;0;0]; pr=[0;l1;0]; pf=[0;l1+l2;0]; pp=[0;l1+l2;l0]; 
Point = [po pr pp pf];
Joint = ['rot'; 'rot'; 'rot'; 'tra'];
Axis = [AxisZ AxisZ AxisZ AxisZ];
Twist = zeros(6,n);
for i = 1:n
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
% Joint SCREWS definition:
TwMag = [Twist; Th];
%
% Links definition:
CM1 = [0; l1/2; 0.2]; CM2 = [0; l1+l2/2; 0.3];
CM3 = [0; l1+l2; 0.4]; CM4 = [0; l1+l2; l0];
IT1 = [0.1; 0.2; 0.3]; IT2 = [0.3; 0.1; 0.5];
IT3 = [0.1; 0.1; 0.1]; IT4 = [0.1; 0.1; 0.1];
mass = [1 1 1 1];
LiMas = [CM1 CM2 CM3 CM4;IT1 IT2 IT3 IT4; mass];
%
% Gravity definition:
PoAcc = [0 0 -10]';
%
%
% We solve the exercise as showed by Murray:
%
% M(t) Inertia Matrix.
al = IT1(3)+(l1/2)^2*mass(1)+l1*l1*mass(2)+l1*l1*mass(3)+l1*l1*mass(4);
be = IT2(3)+IT3(3)+IT4(3)+l2*l2*mass(3)+l2*l2*mass(4)+(l2/2)^2*mass(2);
ga = l1*l2*mass(3)+l1*l2*mass(4)+l1*l2/2*mass(2);
de = IT3(3)+IT4(3);
Mtr1 =[al+be+2*ga*cos(Th(2)) be+ga*cos(Th(2)) de 0];
Mtr2 =[be+ga*cos(Th(2)) be de 0];
MtM =[Mtr1; Mtr2; de de de 0; 0 0 0 mass(4)];
%
% C(t,dt) Coriolis Matrix by Murray
CtdtM = zeros(n);
CtdtM(1,1) = -ga*sin(Th(2))*Thp(2);
CtdtM(1,2) = -ga*sin(Th(2))*(Thp(1)+Thp(2));
CtdtM(2,1) = ga*sin(Th(2))*Thp(1);
%
% N(t) Potential Matrix by Murray
NtM = [0; 0; 0; -mass(4)*PoAcc(3)];
%
% The inverse Dynamics solution (even though is not explicit in Murray's).
TdynMurray = MtM*Thpp' + CtdtM*Thp' + NtM
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