%% Screw Theory - CLASSICAL INVERSE DYNAMICS - KUKA IIWA14.
% KUKA IIWA14 - Home Upright position,
% & Gravity acting in direction -Z (gz).
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
%% E627a_ST24R_ID_KUKAIIWA14_gZ
%
clear
clc
%
% Robot DOF
n = 7;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% KINEMATIC Parameters of the Robot in terms of Screw Theory
po=[0;0;0]; pk=[0;0;0.36]; pr=[0;0;0.78]; pf=[0;0;1.18]; pp=[0;0;1.18];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pk po pr po pf po];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ AxisY AxisZ -AxisY AxisZ AxisY AxisZ];
Twist = zeros(6,n);
for i = 1:n
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
%
% Maximum RANGE for the robot joints rad +/-, (by catalog).
%Thmax = pi/180*[170 120 170 120 170 120 175];
%Thmin = -pi/180*[170 120 170 120 170 120 175];
% Maximum SPEED for the robot joints rad/sec, (by catalog).
%Thpmax = pi/180*[85 85 100 75 130 135 135];
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DYNAMIC Parameters of the Robot at REF HOME POSITION - Only aproximation
% The S Spatial system has the "Z" axis oriented up.
CM1 = [0; -0.03; 0.2775]; CM2 = [0; 0.042; 0.419]; CM3 = [0; 0.03; 0.6945];
CM4 = [0; -0.034; 0.847]; CM5 = [0; -0.021; 1];
CM6 = [0; 0.001; 1.18]; CM7 = [0; 0; 1.28];
IT1 = [0.1; 0.09; 0.02]; IT2 = [0.018; 0.05; 0.044];
IT3 = [0.08; 0.075; 0.01];
IT4 = [0.03; 0.029; 0.01]; IT5 = [0.02; 0.018; 0.005];
IT6 = [0.005; 0.0036; 0.0047]; IT7 = [0.001; 0.001; 0.001];
mass = [4 4 3 2.7 1.7 1.8 0.3];
LiMas = [CM1 CM2 CM3 CM4 CM5 CM6 CM7;IT1 IT2 IT3 IT4 IT5 IT6 IT7; mass];
%
% Potential Action Vector - Gravity definition (i.e., -g direction).
PoAcc = [0 0 -9.81]';
%
% Maximum TORQUE for the robot joints Nm, (by catalog).
%Tdynmax = [320 320 176 176 110 40 40];
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trajectory TARGET defined by JOINT Position, Velocity and Acceleration
% It is only one random target point and the differentiability of the
% position and velocity trajectory is given for granted. Here we are
% concerned with the Dynamic solution for a single trajectory point.
Th = [170*(rand-rand) 120*(rand-rand) 170*(rand-rand) 120*(rand-rand)];
Th = [Th 170*(rand-rand) 120*(rand-rand) 175*(rand-rand)];
Th = Th*pi/180;
Thp = [85*(rand-rand) 85*(rand-rand) 100*(rand-rand)];
Thp = [Thp 75*(rand-rand) 130*(rand-rand) 135*(rand-rand) 135*(rand-rand)];
Thp = Thp*pi/180;
Thpp = [(rand-rand)*Thp(1) (rand-rand)*Thp(2) (rand-rand)*Thp(3)];
Thpp = [Thpp (rand-rand)*Thp(4) (rand-rand)*Thp(5) (rand-rand)*Thp(6)];
Thpp = [Thpp (rand-rand)*Thp(7)];
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