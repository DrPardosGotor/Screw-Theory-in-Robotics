%% "Fcn_ST24R_IK_ABBIRB120_ToolD" Inverse Kinematics IRB120 ToolDown.
%
% Solves rhe INVERSE KINEMATICS for any desired position & orientation
% of the TCP (noap goal) of the ABB IRB120 Robot.
%
% function ThetaSet = Fcn_ST24R_IK_ABBIRB120_ToolD(u)
%
% The inputs "u" (6x1) are composed by the following vectors.
% "traXYZ" (3x1) desired translation for the TcP (noap - "p" goal).
% "rotXYZ" (3x1) desired rotation TcP system (noap - "noa" goal X+Y+Z)
%
% "ThetaSet" (8x6), with rows magnitudes solution for the Robot Joints1..6.
% it respects maximum Magnitude for the robot joints POSITION rad.
% Thmax = pi/180*[170 120 170 120 170 120 175];
%
% Mechanical characteristics of the Robot (AT REF POSITION), which is the
% elbow pose with the TOOL facing DOWN:
% po = Origen for he STATIONARY system of reference.
% pk = point in the crossing of the DOF Th1(rot) & Th2(rot).
% pr = point in the axis of Th3(rot).
% pf = point in the crossing of the DOF Th4(rot), Th5(rot), Th6(rot).
% pp = TcP Tool Center Point
% hst0 = Tool (TcP) configuration (rot+tra) at robot reference position.
%
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
% You should have received a copy of the GNU Leser General Public License
% along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2018/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% Fcn_ST24R_IK_ABBIRB120_ToolD
%
function ThetaSET = Fcn_ST24R_IK_ABBIRB120_ToolD(u) %#codegen
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mechanical characteristics of the Robot (AT REF HOME POSITION):
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joints TWISTS definition and TcP at home.
po=[0;0;0]; pk=[0; 0; 0.290]; pr=[0; 0; 0.560];
pf=[0.302; 0; 0.630]; pp=[0.302; 0; 0.470];
Twist = [0   -0.2900   -0.5600         0   -0.6300         0;
         0         0         0    0.6300         0    0.3020;
         0         0         0         0    0.3020         0;
         0         0         0    1.0000         0         0;
         0    1.0000    1.0000         0    1.0000         0;
    1.0000         0         0         0         0   -1.0000];
%
Hst0 =[-1 0  0 0.302;
        0 1  0 0;
        0 0 -1 0.47;
        0 0  0 1];
%
% Motion RANGE for the robot joints POSITION rad, (by catalog).
Thmax = pi/180*[165 110 70 160 120 400];
Thmin = -pi/180*[165 110 110 160 120 400];
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INVERSE KINEMATICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Homogeneous transformation for the GOAL "noap"
traXYZ = u(1:3); rotXYZ = u(4:6);
noap = rotX2tform(rotXYZ(1))*rotY2tform(rotXYZ(2))*rotZ2tform(rotXYZ(3));
noap(1:3,4)= traXYZ';
%
% Calculate the IK joints solutions (Theta1 - Theta6) using SCREW THEORY
% and basically the PADEN-KAHAN Canonic Subproblems.
Theta = zeros(8,6);
%
% STEP1: Calculate Theta3.
% With "pf" on the axis of E4, E5, E6 and "pk" on the axis of E1, E2.
% We apply (noap*gs0^-1) to "pf" and take the norm of the diffence of that
% resulting point and "pk". Doing so we can calculate Theta3 applying the
% Canonic problem PADEN-KAHAN-THREE, because the screws E4,E5,E6 do not affect
% "pf" and the E1,E2 do not affect the norm of a vector with an end on "pk"
% resulting the problem ||exp(E3^theta3)*pf-pk||=||noap*gs0^-1*pf-pk||
% which by PADEN-KAHAN-THREE has none, one or two solutions for t31 t32.
noapHst0if = noap*(Hst0\[pf; 1]); pk1p = noapHst0if(1:3);
de = norm(pk1p - pk);
t3 = PadenKahanThree(Twist(:,3), pf, pk, de);
%t3(1) = jointmag2limits(t3(1), Thmax(3), Thmin(3), "rot");
%t3(2) = jointmag2limits(t3(2), Thmax(3), Thmin(3), "rot");
Theta(1:4,3) = t3(1); Theta(5:8,3) = t3(2); % put results into Theta
%
% STEP2: Calculate Theta1 & Theta2.
% With "pf" on the axis of E4, E5, E6 we apply (noap*gs0^-1) to "pf" and
% the POE E1..E6 also to "pf" having already known the value for Theta3
% resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws
% E4,E5,E6 do not affect "pf" and the E3 is known (two values),resulting
% the problem exp(E1^theta1)*exp(E2^theta2)*pf' = noap*gs0^-1*pf
% which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions
% t11-t21 & t12-t22 for each value of t3, but we have two, then consider
% for t31 we get t11-t21 & t12-t22 & for t32 we get t13-t23 & t14-t24.
for i = 1:4:5
    pfpt = expScrew([Twist(:,3);Theta(i,3)])*[pf; 1];
    pfp = pfpt(1:3);
    t1t2 = PadenKahanTwo(Twist(:,1),Twist(:,2),pfp,pk1p);
%    t1t2(1,1) = jointmag2limits(t1t2(1,1), Thmax(1), Thmin(1), "rot");
%    t1t2(2,1) = jointmag2limits(t1t2(2,1), Thmax(1), Thmin(1), "rot");
%    t1t2(1,2) = jointmag2limits(t1t2(1,2), Thmax(2), Thmin(2), "rot");
%    t1t2(2,2) = jointmag2limits(t1t2(2,2), Thmax(2), Thmin(2), "rot");
    Theta(i,1:2) = t1t2(1,1:2); Theta(i+1,1:2) = t1t2(1,1:2);
    Theta(i+2,1:2) = t1t2(2,1:2); Theta(i+3,1:2) = t1t2(2,1:2);
end
%
% STEP3: Calculate Theta4 & Theta5.
% With "pp" on the axis of E6 apply E3^-1*E2^-1*E1^-1*noap*gs0^-1 to "pp"
% and also the POE E4*E5*E6 to "pp" knowing already Theta3-Theta2-Theta1,
% resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws
% E6 does not affect "pp" & Th3-Th2-Th1 known (four solutions), the problem
% exp(E4^theta4)*exp(E5^theta5)*pp = pk2p ; with
% pk2p = exp(E3^Th3)^-1*exp(E2^Th2)^-1*exp(E1^Th1)^-1*noap*gs0^-1*pp 
% which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions:
% t31,t21,t11 to t41-t51 & t42-t52 ; t31,t22,t12 to t43-t53 & t44-t54
% t32,t23,t13 to t45-t55 & t46-t56 ; t32,t24,t14 to t47-t57 & t48-t58
%
noapHst0ip = noap*(Hst0\[pp; 1]); 
for i = 1:2:7                     % for the 4 values of t3-t2-t1.
    pk2pt = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0ip;
    pk2pt = (expScrew([Twist(:,2);Theta(i,2)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,3);Theta(i,3)]))\pk2pt;
    pk2p = pk2pt(1:3);
    t4t5 = PadenKahanTwo(Twist(:,4),Twist(:,5),pp,pk2p);
%    t4t5(1,1) = jointmag2limits(t4t5(1,1), Thmax(4), Thmin(4), "rot");
%    t4t5(2,1) = jointmag2limits(t4t5(2,1), Thmax(4), Thmin(4), "rot");
%    t4t5(1,2) = jointmag2limits(t4t5(1,2), Thmax(5), Thmin(5), "rot");
%    t4t5(2,2) = jointmag2limits(t4t5(2,2), Thmax(5), Thmin(5), "rot");
    Theta(i:i+1,4:5) = t4t5;
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
noapHst0io = noap*(Hst0\[po; 1]);
for i = 1:8
    pk3pt = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0io;
    pk3pt = (expScrew([Twist(:,2);Theta(i,2)]))\pk3pt;
    pk3pt = (expScrew([Twist(:,3);Theta(i,3)]))\pk3pt;
    pk3pt = (expScrew([Twist(:,4);Theta(i,4)]))\pk3pt;
    pk3pt = (expScrew([Twist(:,5);Theta(i,5)]))\pk3pt;
    pk3p = pk3pt(1:3);
    t6 = PadenKahanOne(Twist(:,6), po, pk3p);
%    t6 = jointmag2limits(t6, Thmax(6), Thmin(6), "rot");
    Theta(i,6) = t6;
end
%
ThetaSET = Theta;
end
%