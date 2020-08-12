%% Screw Theory - EXAMPLES Inverse Kinematics - Exercise.
% ABB IRB 1600 X120.
%
% The goal of this exercise is to TEST:
% INVERSE KINEMATICS for IRB 1600 X120
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Mechanical characteristics of the Robot (AT REF POSITION):
% po = Origen for he STATIONARY system of reference.
% pk = point on the axis of Th2(rot).
% pr = point in the axis of Th3(rot).
% pf = point in the crossing of the DOF Th4(rot), Th5(rot), Th6(rot).
% pp = TcP Tool Center Point
% hst0 = Tool (TcP) configuration (rot+tra) at robot reference position. 
%
% For checking the quality of this IK solution, this exercise has 3 steps:
% STEP1: Apply ForwardKinemats for the Robot for "whatever" Mag Theta1...6
% so getting a feasible TcP configuration (rot+tra) as Hst.
% STEP2: Calculate the IK solutions by SCREW THEORY management getting
% the magnitud Theta1...6. There can be up to 8 right solutions for this
% problem using this approach (theoretically there is max of 16 solutions).
% STEP3: Test the different solutions applying ForwardKinemats to Robot
% with Theta = [t11...t61; t12...t62; ...; t18...t68] and checking we get
% the same TcP configuration (rot+tra) as Hst.
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
%% E443a_ST24R_IK_ABBIRB1600_PG76PK1
%
clear
clc
%
% n is number of DOF.
n = 6;
Mag = zeros(1,n);
for i = 1:n
    Mag(i) = (rand-rand)*pi; % for testing various Theta1-Theta6
end
%
% Mechanical characteristics of the IRB120 Robot:
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
Hst0 = trvP2tform(pp)*rotY2tform(pi/2);
%
% STEP1: Apply ForwardKinemats to the Robot.
TwMag = [Twist; Mag]; % assign the rand values to joint Theta magnitudes.
HstR = ForwardKinematicsPOE(TwMag);
noap = HstR * Hst0
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IK solution approach PG7+PG6+PK1 subproblems cosecutively.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate the IK solutions Theta using the SCREW THEORY.
Theta_STR6 = zeros(8,n);
tic % start the ticking for calcule the performance of this algorithm.
%
% STEP1: Calculate Theta3.
% With "pf" on the axis of E4, E5, E6 and "pk" on the axis of E1, E2.
% We apply (noap*gs0^-1) to "pf" and take the norm of the diffence of that
% resulting point and "pk". Doing so we can calculate Theta3 applying the
% Canonic problem PADEN-KAHAN-THREE, because the screws E4,E5,E6 do not affect
% "pf" and the E1,E2 do not affect the norm of a vector with an end on "pk"
% resulting the problem ||exp(E3^theta3)*pf-pk||=||noap*gs0^-1*pf-pk||
% which by PARDOS-GOTOR-SEVEN has none, two or four solutions for t123.
noapHst0if = noap*(Hst0\[pf; 1]); pkp = noapHst0if(1:3);
t123 = PardosGotorSeven(Twist(:,1), Twist(:,2), Twist(:,3), pf, pkp);
Theta_STR6(1,1:3) = t123(1,:);
Theta_STR6(2,1:3) = t123(1,:);
Theta_STR6(3,1:3) = t123(2,:);
Theta_STR6(4,1:3) = t123(2,:);
Theta_STR6(5,1:3) = t123(3,:);
Theta_STR6(6,1:3) = t123(3,:);
Theta_STR6(7,1:3) = t123(4,:);
Theta_STR6(8,1:3) = t123(4,:);
%
% STEP2: Calculate Theta4 & Theta5.
% With "pp" on the axis of E6 apply E3^-1*E2^-1*E1^-1*noap*gs0^-1 to "pp"
% and also the POE E4*E5*E6 to "pp" knowing already Theta3-Theta2-Theta1,
% resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws
% E6 does not affect "pp" & Th3-Th2-Th1 known (four solutions), the problem
% exp(E4^theta4)*exp(E5^theta5)*pp = pk2p ; with
% pk2p = exp(E3^Th3)^-1*exp(E2^Th2)^-1*exp(E1^Th1)^-1*noap*gs0^-1*pp 
% which by PARDOS-GOTOR-SIX has none, one or two DOUBLE solutions:
%
noapHst0ip = noap*(Hst0\[pp; 1]); 
for i = 1:2:7                     % for the 4 values of t3-t2-t1.
    pk2pt = (expScrew([Twist(:,1);Theta_STR6(i,1)]))\noapHst0ip;
    pk2pt = (expScrew([Twist(:,2);Theta_STR6(i,2)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,3);Theta_STR6(i,3)]))\pk2pt;
    pk2p = pk2pt(1:3);
    t4t5 = PardosGotorSix(Twist(:,4),Twist(:,5),pp,pk2p);
    Theta_STR6(i:i+1,4:5) = t4t5;
end
%
% STEP3: Calculate Theta6.
% With "po" not in the axis of E6 apply E5^-1...*E1^-1*noap*gs0^-1 to "po"
% and applying E6 to "po" knowing already Theta5...Theta1 (8 solutions),
% resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
% exp(E6^theta6)*po = pk3p ; with
% pk3p = exp(E5^Th5)^-1*...*exp(E1^Th1)^-1*noap*gs0^-1*po 
% which by PADEN-KAHAN-ONE has none or one solution. Then for all
% Th5-Th4-Th3-Th2-Th1 known (eight solutions) we get t61...t68:
%
noapHst0io = noap*(Hst0\[po; 1]);
for i = 1:size(Theta_STR6,1)
    pk2pt = (expScrew([Twist(:,1);Theta_STR6(i,1)]))\noapHst0io;
    pk2pt = (expScrew([Twist(:,2);Theta_STR6(i,2)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,3);Theta_STR6(i,3)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,4);Theta_STR6(i,4)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,5);Theta_STR6(i,5)]))\pk2pt;
    pk3p = pk2pt(1:3);
    Theta_STR6(i,6) = PadenKahanOne(Twist(:,6), po, pk3p);
end
%
Theta_STR6
%
tIK1 = round(toc*1000,1);
time_IK_STR1 = ['Time to solve IK Screw Theory ', num2str(tIK1),' ms']
%
%
% STEP3: Test the different solutions applying ForwardKinemats to Robot
for i = 1:size(Theta_STR6,1)
    TwMagi = [Twist; Theta_STR6(i,:)];
    HstRi = ForwardKinematicsPOE(TwMagi);
    i
    noapi = HstRi * Hst0
end
%
% Check that TcP POSE (rot+tra) as Hst is OK for all Theta values
%