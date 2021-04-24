%% Screw Theory - EXAMPLES Inverse Kinematics.
% ABB IRB120 (TOOLDOWN).
% Algorithm applied: PG5 + PG4 + PG6 + PK1.
%
% The goal of this exercise is to TEST:
% INVERSE KINEMATICS for IRB120 ToolDown POSE
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Mechanical characteristics of the Robot (AT REF POSITION):
% po = Origen for he STATIONARY system of reference.
% pk = point in the crossing of the DOF Th1(rot) & Th2(rot).
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
%% E441d_STR24R_IK_ABBIRB120_PG546PK1
%
clear
clc
% n is number of DOF.
n = 6;
Mag = zeros(1,n);
for i = 1:6
    Mag(i) = (rand-rand)*pi; % for testing various Theta1-Theta6
end
%
% Mechanical characteristics of the IRB120 Robot:
po=[0;0;0]; pk=[0; 0; 0.290]; pr=[0; 0; 0.560];
pf=[0.302; 0; 0.630]; pp=[0.302; 0; 0.470];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pk pr pf pf pf];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ AxisY AxisY AxisX AxisY -AxisZ];
for i = 1:6
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp)*rotY2tform(pi);
%
% STEP1: Apply ForwardKinemats to the Robot.
TwMag = [Twist; Mag]; % assign the rand values to joint Theta magnitudes.
HstR = ForwardKinematicsPOE(TwMag);
noap = HstR * Hst0
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IK solution approach PG5+PG4+PG6+PK1 subproblems cosecutively.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate the IK solutions Theta using the SCREW THEORY.
Theta_STR4 = zeros(8,n);
tic % start the ticking for calcule the performance of this algorithm.
%
% STEP1: Calculate Theta1.
% With "pf" on the axis of E4, E5, E6. We apply (noap*hs0^-1) to "pf"
% Doing so we get Theta1 applying the Canonic problem PADEN-KAHAN-ONE,
% because the screws E4,E5,E6 do not affect "pf" for being on their axes
% and the E2,E3 do not change the plane where "pf" moves, and so do not
% affect the calculation for Theta1 resulting the problem 
% "exp(E1^theta1)*pf = noap*hs0^-1*pf" by PK1.
% which has two solution for t1 by PARDOS-GOTOR-FIVE.
noapHst0if = noap*(Hst0\[pf; 1]); pk1 = noapHst0if(1:3);
t1 = PardosGotorFive(Twist(:,1), pf, pk1);
% prepare Theta for next calculation
Theta_STR4(1:4,1) = t1(1);
Theta_STR4(5:8,1) = t1(2);
%
% STEP2: Calculate Theta2 & Theta3.
% With "pf" on the axis of E4, E5, E6 we apply (noap*hs0^-1) to "pf" and
% the POE E1..E6 also to "pf" having already known the value for Theta1
% resulting exactly a Canonic problem PARDOS-FOUR, because the screws
% E4,E5,E6 do not affect "pf" and the E1 is known,resulting the problem
% exp(E2^theta2)*exp(E3^theta3)*pf = exp(E1^Th1)^-1*noap*gs0^-1*pf = pk1p
% which by PARDOS-FOUR has none, one or two DOUBLE solutions.
% t21-t31 & t22-t32 for each value of t11
%
for i = 1:4:5
    E1inoapHst0if = (expScrew([Twist(:,1);Theta_STR4(i,1)]))\noapHst0if;
    pk2 = E1inoapHst0if(1:3);
    t2t3 = PardosGotorFour(Twist(:,2),Twist(:,3),pf,pk2);
    Theta_STR4(i,2:3) = t2t3(1,:); 
    Theta_STR4(i+1,2:3) = t2t3(1,:); 
    Theta_STR4(i+2,2:3) = t2t3(2,:);
    Theta_STR4(i+3,2:3) = t2t3(2,:);
end
%
% STEP3: Calculate Theta4 & Theta5.
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
    pk2pt = (expScrew([Twist(:,1);Theta_STR4(i,1)]))\noapHst0ip;
    pk2pt = (expScrew([Twist(:,2);Theta_STR4(i,2)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,3);Theta_STR4(i,3)]))\pk2pt;
    pk2p = pk2pt(1:3);
    t4t5 = PardosGotorSix(Twist(:,4),Twist(:,5),pp,pk2p);
    Theta_STR4(i:i+1,4:5) = t4t5; 
end
%
% STEP4: Calculate Theta6.
% With "po" not in the axis of E6 apply E5^-1...*E1^-1*noap*gs0^-1 to "po"
% and applying E6 to "po" knowing already Theta5...Theta1 (8 solutions),
% resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
% exp(E6^theta6)*po = pk3p ; with
% pk3p = exp(E5^Th5)^-1*...*exp(E1^Th1)^-1*noap*gs0^-1*po 
% which by PADEN-KAHAN-ONE has none or one solution. Then for all
% Th5-Th4-Th3-Th2-Th1 known (eight solutions) we get t61...t68:
%
noapHst0io = noap*(Hst0\[po; 1]);
for i = 1:size(Theta_STR4,1)
    pk2pt = (expScrew([Twist(:,1);Theta_STR4(i,1)]))\noapHst0io;
    pk2pt = (expScrew([Twist(:,2);Theta_STR4(i,2)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,3);Theta_STR4(i,3)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,4);Theta_STR4(i,4)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,5);Theta_STR4(i,5)]))\pk2pt;
    pk3p = pk2pt(1:3);
    Theta_STR4(i,6) = PadenKahanOne(Twist(:,6), po, pk3p);
end
%
%
Theta_STR4
%
tIK4 = round(toc*1000,1);
time_IK_STR4 = ['Time to solve IK Screw Theory ', num2str(tIK4),' ms']
%
%
% STEP3: Test the different solutions applying ForwardKinemats to Robot
for i = 1:size(Theta_STR4,1)
    TwMagi = [Twist; Theta_STR4(i,:)];
    HstRi = ForwardKinematicsPOE(TwMagi);
    i
    noapi = HstRi * Hst0
end
%
% Check that TcP POSE (rot+tra) as Hst is OK for all Theta values
%