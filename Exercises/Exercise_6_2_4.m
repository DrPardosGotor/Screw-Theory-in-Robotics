%% Screw Theory in Robotics
% An Illustrated and Practicable Introduction to Modern Mechanics
% by CRC Press
% © 2022 Jose M Pardos-Gotor
%
%% Ch6 - INVERSE DYNAMICS.
%
% Exercise 6.2.4: ABB IRB1600 - Lagrange ID.
% Home position Tool on X.
% & Gravity acting in direction -Z (gz).
%
% The goal of this exercise is to prove the DYNAMICS
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
% M(t)*ddt + C(t,dt)*dt + N(t,dt) = T 
%
% Copyright (C) 2003-2021, by Dr. Jose M. Pardos-Gotor.
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
% Revision 1.1  2021/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% HOME Code.
%
clear
clc
%
% Robot DOF
n = 6;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% KINEMATIC Parameters of the Robot in terms of Screw Theory
po=[0;0;0]; pk=[0.15;0;0.4865]; pr=[0.15;0;0.9615];
pf=[0.75;0;0.9615]; pp=[0.9;0;0.9615];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pk pr pf pf pf];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ AxisY AxisY AxisX AxisY AxisX];
Twist = zeros(6,n);
for i = 1:n
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
%
% Motion RANGE for the robot joints POSITION rad, (by catalog).
% Thmax = pi/180*[180 150 65 190 115 400];
% Thmin = -pi/180*[180 90 245 190 115 400];
% Maximum SPEED for the robot joints rad/sec, (by catalog).
% Thpmax = pi/180*[180 180 185 385 400 460];
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DYNAMIC Parameters of the Robot at REF HOME POSITION - Only aproximation
% The S Spatial system has the "Z" axis oriented up.
CM1 = [0.15; 0; 0.5]; CM2 = [0.15; -0.15; 0.75]; CM3 = [0.2; 0; 0.96];
CM4 = [0.55; 0; 0.96]; CM5 = [0.75; 0; 0.96]; CM6 = [0.8; 0; 0.96];
IT1 = [0.2; 0.2; 0.3]; IT2 = [0.1; 0.1; 0.2]; IT3 = [0.2; 0.1; 0.1];
IT4 = [0.1; 0.1; 0.1]; IT5 = [0.1; 0.1; 0.1]; IT6 = [0.1; 0.3; 0.3];
mass = [75 35 25 20 10 5];
LiMas = [CM1 CM2 CM3 CM4 CM5 CM6;IT1 IT2 IT3 IT4 IT5 IT6; mass];
%
% Potential Action Vector - Gravity definition (i.e., -g direction).
PoAcc = [0 0 -9.81]';
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trajectory TARGET defined by JOINT Position, Velocity and Acceleration
% It is only one random target point and the differentiability of the
% position and velocity trajectory is given for granted. Here we are
% concerned with the Dynamic solution for a single trajectory point.
Th = [180*(rand-rand) 150*rand-rand*90 65*rand-rand*245];
Th = [Th 190*(rand-rand) 115*(rand-rand) 400*(rand-rand)];
Th = Th*pi/180;
Thp = [180*(rand-rand) 180*(rand-rand) 185*(rand-rand)];
Thp = [Thp 385*(rand-rand) 400*(rand-rand) 460*(rand-rand)];
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