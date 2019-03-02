%% Screw Theory - DYNAMICS - Exercise ABB IRB 910SC.
%
% The goal of this exercise is to prove the DYNAMICS
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
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
% http://www.preh
%
% CHANGES:
% Revision 1.1  2019/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% E873_STR_DYNAMICS_ABBIRB910SC
%
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
% Mechanical characteristics of the Robot:
PoAcc = [0 -9.8 0]';
l1 = 0.4; l2 = 0.25; l3 = 0.125;
CM1 = [0.2; 0.2; 0]; CM2 = [0.5; 0.3; 0];
CM3 = [0.65; 0.2; 0]; CM4 = [0.65; 0.125; 0];
IT1 = [0.1; 0.3; 0.2]; IT2 = [0.1; 0.5; 0.3];
IT3 = [0.1; 0; 0.1]; IT4 = [0.1; 0.1; 0.1];
mass = [2 5 1 0.1];
LiMas = [CM1 CM2 CM3 CM4;IT1 IT2 IT3 IT4; mass];
%
po=[0;0;0]; pr=[l1;0;0]; pf=[l1+l2;0;0]; pp=[l1+l2;l3;0]; 
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pr pf pp];
Joint = ['rot'; 'rot'; 'tra'; 'rot'];
Axis = [AxisY AxisY -AxisY -AxisY];
%
Twist = zeros(6,n);
for i = 1:n
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
TwMag = [Twist; Th];
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