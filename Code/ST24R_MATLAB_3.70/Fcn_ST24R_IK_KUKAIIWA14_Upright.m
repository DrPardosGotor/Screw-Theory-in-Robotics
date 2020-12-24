%% Function "Fcn_ST24R_IIWA14_IK_Upright" IIWA14 INVERSE KINEMATICS
% KUKA IIWA 14 R820 - Robot HOME pose is Upright.
%
% It solves the INVERSE KINEMATICS for any desired Tool POSE 
% (i.e. position & orientation of the TCP system (noap goal) of the Robot.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% function ThetaSet = Fcn_ST24R_IIWA14_IK_Upright(u)
%
% The inputs "u" (7x1) are composed by the following vectors.
% "traXYZ" (3x1) desired translation for the TcP (noap - "p" goal).
% "rotXYZ" (3x1) desired rotation TcP system (noap - "noa" goal X+Y+Z)
%
% "ThetaSet" (16x7), with rows magnitudes solution for Robot Joints 1..7.
% it respects maximum Magnitude for the robot joints POSITION rad.
% Thmax = pi/180*[170 120 170 120 170 120 175];
%
% Mechanical characteristics of the Robot (AT REF HOME POSITION):
% The S Spatial system has the "Z" axis up (i.e., -g direction).
% po = Origen for he STATIONARY system of reference.
% pk = point in the crossing of the DOF Th1(rot) & Th2(rot) & Th3(rot).
% pr = point in the axis of Th4(rot) Th5(rot).
% pf = point in the crossing of the DOF Th5(rot), Th6(rot), Th7(rot).
% pp = TcP Tool Center Point
% hst0 = Tool (TcP) POSE configuration (rot+tra) at reference position. 
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
% You should have received a copy of the GNU Leser General Public License
% along with ST24R. If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2019/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% Fcn_ST24R_IK_KUKAIIWA14_Upright
%
function ThetaSet = Fcn_ST24R_IK_KUKAIIWA14_Upright(u) % #codegen
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mechanical characteristics of the Robot (AT REF HOME POSITION):
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joints TWISTS definition and TcP at home.
pk=[0;0;0.36]; pf=[0;0;1.18]; pp=[0;0;1.38];
Twist = [0   -0.3600         0    0.7800         0   -1.1800         0;
         0         0         0         0         0         0         0;
         0         0         0         0         0         0         0;
         0         0         0         0         0         0         0;
         0    1.0000         0   -1.0000         0    1.0000         0;
    1.0000         0    1.0000         0    1.0000         0     1.000];
Hst0 = [1 0 0 0; 0 1 0 0; 0 0 1 1.38; 0 0 0 1];
%
% Motion RANGE for the robot joints POSITION rad, (by catalog).
Thmax = pi/180*[170 120 170 120 170 120 175];
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INVERSE KINEMATICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Homogeneous transformation for the GOAL "noap"
traXYZ = u(1:3); rotXYZ = u(4:6);
noap = rotX2tform(rotXYZ(1))*rotY2tform(rotXYZ(2))*rotZ2tform(rotXYZ(3));
noap(1:3,4)= traXYZ';
%
% Calculate the IK joints solutions (Theta1 - Theta7) using SCREW THEORY
% and basically the PADEN-KAHAN Canonic Subproblems. The algorithm is 
% proposed by Dr. Pardos-Gotor and gives a set of 16 possible solutions.
Theta = zeros(16,7);
%
% STEP1: Calculate two "feasible and logical" inputs for t1 and t3.
% First, as this is REDUNDANT robot with 7DoF and theoretical infinite
% solutions, we reduce the number of possibilies to 16, and for doing so
% we give some solution INPUTS for t1 and t3 based on some other criteria,
% and then well get 8 solutions for each of these INPUTS for the rest 6DoF.
% For this function we choose t1 and t3 to be the magnitudes for orientate
% these joint towards the TcP, and for doing so we use PADEN-KAHAN-ONE.
t1 = PadenKahanOne(Twist(:,1), [1; 0; 0], traXYZ');
t1 = jointmag2limits(t1, Thmax(1), -Thmax(1), "rot");
%
t3 = PadenKahanOne(Twist(:,3), [1; 0; 0], traXYZ');
t3 = jointmag2limits(t3, Thmax(3), -Thmax(3), "rot");
Theta(1:8,3) = t3; Theta(9:16,1) = t1;
%
% STEP2-1: Calculate Theta4.
% With "pf" on the axis of E5, E6, E7 and "pk" on the axis of E1, E2, E3.
% We apply (noap*hst0^-1) to "pf" and take the norm of the diffence of that
% resulting point and "pk". Doing so we can calculate Theta4 applying the
% Canonic problem PADEN-KAHAN-THREE, because the screws E4,E5,E6 do not 
% affect "pf" and the E1,E2,E3 do not affect the norm of a vector with an
% end on "pk" resulting the problem 
% ||exp(E4^t4)*pf-pk|| = ||noap*hst0^-1*pf-pk|| = ||pk1p-pk|| = de
% which by PADEN-KAHAN-THREE has none, one or two solutions
% t401 & t402.
noapHst0if = noap*(Hst0\[pf; 1]); pk1p = noapHst0if(1:3);
de = norm(pk1p - pk);
t4 = PadenKahanThree(Twist(:,4), pf, pk, de);
t4(1) = jointmag2limits(t4(1), Thmax(4), -Thmax(4), "rot");
t4(2) = jointmag2limits(t4(2), Thmax(4), -Thmax(4), "rot");
Theta(1:4,4) = t4(1); Theta(9:12,4) = t4(1);
Theta(5:8,4) = t4(2); Theta(13:16,4) = t4(2);
%
% STEP2-2: Calculate Theta1 & Theta2, knowing Theta3 (t3in).
% With "pf" on the axis of E5, E6, E7 we apply (noap*hsts0^-1) to "pf" and
% the POE E1..E7 also to "pf" having already known the value for t4 & t3
% resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws
% E5,E6,E7 do not affect "pf" and the E4 & E3 are known,resulting
% the problem exp(E1^theta1)*exp(E2^theta2)*pf' = noap*hst0^-1*pf = pk1p
% which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions
% t401 & t3in => t201-t101 & t202-t102.
% t402 & t3in => t203-t103 & t204-t104. 
for i = 1:4:5
    pf1pt = expScrew([Twist(:,3);Theta(i,3)])*expScrew([Twist(:,4);Theta(i,4)])*[pf; 1];
    pf1p = pf1pt(1:3);
    t1t2 = PadenKahanTwo(Twist(:,1),Twist(:,2),pf1p,pk1p);
    t1t2(1,1) = jointmag2limits(t1t2(1,1), Thmax(1), -Thmax(1), "rot");
    t1t2(2,1) = jointmag2limits(t1t2(2,1), Thmax(1), -Thmax(1), "rot");
    t1t2(1,2) = jointmag2limits(t1t2(1,2), Thmax(2), -Thmax(2), "rot");
    t1t2(2,2) = jointmag2limits(t1t2(2,2), Thmax(2), -Thmax(2), "rot");
    Theta(i,1:2) = t1t2(1,1:2); Theta(i+1,1:2) = t1t2(1,1:2);
    Theta(i+2,1:2) = t1t2(2,1:2); Theta(i+3,1:2) = t1t2(2,1:2);
end
%
% STEP2-3: Calculate Theta2 & Theta3, knowing Theta1 (t1in).
% With "pf" on the axis of E5,E6,E7 we apply E1^-1*(noap*hsts0^-1) to "pf"
% and POE E2..E7 also to "pf" having already known the value for t4 & t1
% resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws
% E5,E6,E7 do not affect "pf" and the E4 is known,resulting
% the problem exp(E2^t2)*exp(E3^t3)*pf'' = E1^-1*noap*hst0^-1*pf = pk2p
% which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions
% t401 & t1in => t301-t205 & t302-t206.
% t402 & t1in => t303-t207 & t304-t208.
%
for i = 9:4:13
    pf2pt = expScrew([Twist(:,4);Theta(i,4)])*[pf; 1];
    pf2p = pf2pt(1:3);
    pk3pt = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0if;
    pk3p = pk3pt(1:3);
    t2t3 = PadenKahanTwo(Twist(:,2),Twist(:,3),pf2p,pk3p);
    t2t3(1,1) = jointmag2limits(t2t3(1,1), Thmax(2), -Thmax(2), "rot");
    t2t3(2,1) = jointmag2limits(t2t3(2,1), Thmax(2), -Thmax(2), "rot");
    t2t3(1,2) = jointmag2limits(t2t3(1,2), Thmax(3), -Thmax(3), "rot");
    t2t3(2,2) = jointmag2limits(t2t3(2,2), Thmax(3), -Thmax(3), "rot");
    Theta(i,2:3) = t2t3(1,1:2); Theta(i+1,2:3) = t2t3(1,1:2);
    Theta(i+2,2:3) = t2t3(2,1:2); Theta(i+3,2:3) = t2t3(2,1:2);
end
%
% STEP2-4: Calculate Theta5 & Theta6.
% With "pp" on the axis of E7 apply E4^-1*E3^-1**E2^-1*E1^-1*noap*hst0^-1
% to "pp" and also to the POE E5*E6*E7 knowing already t4,t3,t2,t1.
% resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws
% E7 does not affect "pp" and the result is the problem
% exp(E5^theta5)*exp(E6^theta6)*pp = pk3p ; with
% pk3p = exp(E4^t4)^-1*exp(E3^t3)^-1*exp(E2^t2)^-1*exp(E1^t1)^-1*noap*hst0^-1*pp 
% which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions:
% t101-t201-t3in-t401 => t501-t601 & t502-t602
% t102-t202-t3in-t401 => t503-t603 & t504-t604
% t103-t203-t3in-t402 => t505-t605 & t506-t606
% t104-t204-t3in-t402 => t507-t607 & t508-t608
% t1in-t205-t301-t401 => t509-t609 & t510-t610
% t1in-t206-t302-t401 => t511-t611 & t512-t612
% t1in-t207-t303-t402 => t513-t613 & t514-t614
% t1in-t208-t304-t402 => t515-t615 & t516-t616
%
noapHst0ip = noap*(Hst0\[pp; 1]); 
for i = 1:2:15
    pk3pt = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0ip;
    pk3pt = (expScrew([Twist(:,2);Theta(i,2)]))\pk3pt;
    pk3pt = (expScrew([Twist(:,3);Theta(i,3)]))\pk3pt;
    pk3pt = (expScrew([Twist(:,4);Theta(i,4)]))\pk3pt;
    pk3p = pk3pt(1:3);
    t5t6 = PadenKahanTwo(Twist(:,5),Twist(:,6),pp,pk3p);
    t5t6(1,1) = jointmag2limits(t5t6(1,1), Thmax(5), -Thmax(5), "rot");
    t5t6(2,1) = jointmag2limits(t5t6(2,1), Thmax(5), -Thmax(5), "rot");
    t5t6(1,2) = jointmag2limits(t5t6(1,2), Thmax(6), -Thmax(6), "rot");
    t5t6(2,2) = jointmag2limits(t5t6(2,2), Thmax(6), -Thmax(6), "rot");
    Theta(i:i+1,5:6) = t5t6;
end
%
% STEP2-5: Calculate Theta7.
% With "po" not in the axis of E7 apply E6^-1...*E1^-1*noap*gs0^-1 to "po"
% and applying E7 to "po" knowing already Theta6...Theta1,
% resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
% exp(E7^theta7)*po = pk4p ; with
% pk4p = exp(E6^Th6)^-1*...*exp(E1^Th1)^-1*noap*hst0^-1*po 
% which by PADEN-KAHAN-ONE has none or one solution. Then for all
% Th6-Th5-Th4-Th3-Th2-Th1 we get a Th7 = t701...t716:
%
noapHst0io = noap*(Hst0\[[1; 0; 0]; 1]);
for i = 1:16
    pk4pt = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0io;
    pk4pt = (expScrew([Twist(:,2);Theta(i,2)]))\pk4pt;
    pk4pt = (expScrew([Twist(:,3);Theta(i,3)]))\pk4pt;
    pk4pt = (expScrew([Twist(:,4);Theta(i,4)]))\pk4pt;
    pk4pt = (expScrew([Twist(:,5);Theta(i,5)]))\pk4pt;
    pk4pt = (expScrew([Twist(:,6);Theta(i,6)]))\pk4pt;
    pk4p = pk4pt(1:3);
    t7 = PadenKahanOne(Twist(:,7), [1; 0; 0], pk4p);
    t7 = jointmag2limits(t7, Thmax(7), -Thmax(7), "rot");
    Theta(i,7) = t7;
end
%
ThetaSet = Theta;
end
%