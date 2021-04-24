%% Screw Theory - CLASSICAL INVERSE DYNAMICS - ABB IRB120.
% ABB IRB120 Home position Elbow & Tool Up
% & Gravity acting in direction -Y (gy).
%
% The goal of this exercise is to prove the DYNAMICS
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
% M(t)*ddt + C(t,dt)*dt + N(t,dt) = T
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
% http://www.
%
% CHANGES:
% Revision 1.1  2020/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% E622a_ST24R_ID_ABBIRB120_ToolUp_CLA
%
clear
clc
%
% Robot DOF
n = 6;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% KINEMATIC Parameters of the Robot in terms of Screw Theory
po=[0;0;0]; pk=[0;0.290;0]; pr=[0;0.560;0];
pf=[0.302;0.630;0]; pp=[0.302;0.790;0];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [pk pk pr pf pf pp];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisY AxisZ AxisZ AxisX AxisZ AxisY];
Twist = zeros(6,n);
for i = 1:6
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp)*rotX2tform(-pi/2)*rotZ2tform(pi);
%
% Motion RANGE for the robot joints POSITION rad, (by catalog).
% Thmax = pi/180*[165 110 70 160 120 400];
% Thmin = -pi/180*[165 110 110 160 120 400];
% Maximum SPEED for the robot joints rad/sec, (by catalog).
% Thpmax = pi/180*[250 250 250 320 320 420];
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DYNAMIC Parameters of the Robot at REF HOME POSITION - Only aproximation
% The S Spatial system has the "Z" axis oriented up.
CM1 = [0; 0.29; 0]; CM2 = [0; 0.425; 0]; CM3 = [0; 0.63; 0];
CM4 = [0.2; 0.63; 0]; CM5 = [0.302; 0.63; 0]; CM6 = [0.302; 0.73; 0];
IT1 = [0.1; 0.2; 0.3]; IT2 = [0.3; 0.1; 0.5]; IT3 = [0.1; 0.1; 0.1];
IT4 = [0.1; 0.2; 0.3]; IT5 = [0.1; 0.1; 0.1]; IT6 = [0.1; 0.1; 0.1];
mass = [7 6 5 4 2 1];
LiMas = [CM1 CM2 CM3 CM4 CM5 CM6;IT1 IT2 IT3 IT4 IT5 IT6; mass];
%
% Potential Action Vector - Gravity definition (i.e., -g direction).
PoAcc = [0 -9.81 0]';
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trajectory TARGET defined by JOINT Position, Velocity and Acceleration
% It is only one random target point and the differentiability of the
% position and velocity trajectory is given for granted. Here we are
% concerned with the Dynamic solution for a single trajectory point.
Th = [165*(rand-rand) 110*(rand-rand) 70*(rand-rand)];
Th = [Th 160*(rand-rand) 120*(rand-rand) 400*(rand-rand)];
Th = Th*pi/180;
Thp = [250*(rand-rand) 250*(rand-rand) 250*(rand-rand)];
Thp = [Thp 320*(rand-rand) 320*(rand-rand) 420*(rand-rand)];
Thp = Thp*pi/180;
Thpp = [(rand-rand)*Thp(1) (rand-rand)*Thp(2) (rand-rand)*Thp(3)];
Thpp = [Thpp (rand-rand)*Thp(4) (rand-rand)*Thp(5) (rand-rand)*Thp(6)];
TwMag = [Twist; Th];
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CLASSICAL SCREW THEORY INVERSE DYNAMICS solution
%
% First with the SYMBOLIC algorithm.
% This is with the new symbolic solution for N(t).
tic;
% M(t) Inertia matrix by the use of Jsl LINK TOOL Jacobian.
MtST24RJsl = MInertiaJsl(TwMag,LiMas);
%
% C(t,dt) Coriolis matrix by the use of Aij Adjoint transformation.
CtdtST24RAij = CCoriolisAij(TwMag,LiMas,Thp);
%
% N(t) Potential by the use of the new GRAVITY SYMBOLIC matrix.
NtST24RSym = NPotentialDifSym(TwMag,LiMas,PoAcc);
%
% Inverse Dynamics solution for the joint TORQUES T.
TdynST24RSym = MtST24RJsl*Thpp' + CtdtST24RAij*Thp' + NtST24RSym
toc
%
%
% Second with the TWIST algorithm.
% This is with the new gravity twist matrix for N(t).
tic;
% M(t) Inertia matrix by the use of the NEW ADJOINT Aij transformation.
MtST24RAij = MInertiaAij(TwMag,LiMas);
%
% C(t,dt) Coriolis matrix by the use of Aij Adjoint transformation.
CtdtST24RAij = CCoriolisAij(TwMag,LiMas,Thp);
%
% N(t) Potential by the use of the new GRAVITY TWIST Matrix.
NtST24RAij = NPotentialAij(TwMag,LiMas,PoAcc);
%
% Inverse Dynamics solution for the joint TORQUES T.
TdynST24RAij = MtST24RAij*Thpp' + CtdtST24RAij*Thp' + NtST24RAij
toc
%
%
% Third with the WRENCH algorithm.
% This is with the new gravity wrench matrix for N(t).
tic;
% M(t) Inertia matrix by the use of Jsl LINK TOOL Jacobian.
MtST24RJsl = MInertiaJsl(TwMag,LiMas);
%
% C(t,dt) Coriolis matrix by the use of Aij Adjoint transformation.
CtdtST24RAij = CCoriolisAij(TwMag,LiMas,Thp);
%
% N(t) Potential by the use of the new GRAVITY WRENCH Matrix.
NtST24RWre = NPotentialWre(TwMag,LiMas,PoAcc);
%
% Inverse Dynamics solution for the joint TORQUES T.
TdynST24RWre = MtST24RJsl*Thpp' + CtdtST24RAij*Thp' + NtST24RWre
toc
%