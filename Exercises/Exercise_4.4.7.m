%% Screw Theory - EXAMPLES Inverse Kinematics.
% % COLLABORATIVE robot - UNIVERSAL UR16e.
% Algorithm applied: PG5 + PG3 + PK1 + PK8.
%
% The goal of this exercise is to TEST:
% INVERSE KINEMATICS for UR16e.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Mechanical characteristics of the Robot (AT REF POSITION):
% po = Origen for he STATIONARY system of reference.
% pk = point in the crossing of the DOF Th1(rot) & Th2(rot).
% pr = point in the axis of Th3(rot).
% pf = point in the crossing of the DOF Th4(rot), Th5(rot).
% pg = point in the crossing of the DOF Th5(rot), Th6(rot).
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
%% E446a_ST24R_IK_UR16e_PG53PK1PG8
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
Mag
%
% Mechanical characteristics of the Robot:
po=[0; 0; 0]; pk=[0; 0; 0.181]; pr=[0.478; 0; 0.181];
pf=[0.838; 0.174; 0.181];
pg=[0.838; 0.174; 0.061]; pp=[0.838; 0.364; 0.061];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pk pr pf pg pp];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ AxisY AxisY AxisY -AxisZ AxisY];
Twist = zeros(6,6);
for i = 1:6
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp)*rotX2tform(-pi/2)*rotZ2tform(pi);
%
%
% STEP1: Apply ForwardKinemats to the Robot.
TwMag = [Twist; Mag]; % assign the rand values to joint Theta magnitudes.
noap = ForwardKinematicsPOE(TwMag) * Hst0
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IK solution approach PG5+PG3+PK1+PG8 subproblems cosecutively.
% ATTENTION, because this algorithm can give two, four or eight correct
% solutions, as a function of the target and singularities, then
% You must check with the FK which one is right out of the eight.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate the IK solutions Theta using the SCREW THEORY.
Theta_STR4 = zeros(8,n);
tic % start the ticking for calcule the performance of this algorithm.
%
% STEP1: Calculate Theta1.
% With "pg" on the axis of E5, E6. We apply (noap*hs0^-1) to "pg",
% because the screws E5,E6 do not affect "pg" for being on their axes
% and the E2,E3,E4 do not change the plane where "pg" moves, and so do not
% affect the calculation for Theta1 resulting the problem 
% "exp(E1^theta1)*pg = noap*hs0^-1*pg"
% which has one solution for t1 by PARDOS-GOTOR-FIVE.
% Pay attention to the mechanical configuration of the robot, because
% to comply with t12, we must take into consideration the displacement of
% point "pg" (i.e., d4) out of the robot plane of motion (the plane which 
% includes points o, k and r. Besides, for t12 we must add "pi" to the 
% calculation, otherwise it is impossible to reach the point g.
noapHst0ig = noap*(Hst0\[pg; 1]); pk1 = noapHst0ig(1:3);
t1 = PardosGotorFive(Twist(:,1), pg, pk1);
v1 = Twist(1:3,1); w1 = Twist(4:6,1); r1 = cross(w1,v1)/(norm(w1)^2);
v = pk1 - r1; vw1 = w1*w1'*v; vp1 = v-vw1; nvp = norm(vp1);
u = pg - r1; uw1 = w1*w1'*u; up1 = u-uw1; nup = norm(up1);
t11 = t1(1) - asin(pg(2)/nvp) + asin(pg(2)/nup);
t12 = t1(2) + asin(pg(2)/nvp) + asin(pg(2)/nup);
Theta_STR4(1:4,1) = real(t11);
Theta_STR4(5:8,1) = real(t12);
%
% STEP2: Calculate Theta5.
% With "pp" not in the axis of E5 apply E1^-1*noap*gs0^-1 to "pp"
% E6 does not affect "pp" because is in its axis. Then applying E5 to "pp" 
% knowing already Theta1 gives a new point "k2p", but we must consider the
% effect of E2, E3 and E4. These three parallel rotations only make point
% "k2p" move along axis "X" a certain amount. To calculate this magnitude
% we use the Pardos-Gotor-Three (point translation to a given distance to 
% another point). Where the point is "k2p" the distance is the radius of 
% the joint rotation Theta5 (i.e., norm(pp-pg)) to the point "pg"). Solving
% this PG3, we obtain point "k2",  resulting exactly a Canonic problem 
% PADEN-KAHAN-ONE, which has none or one solution for any Th1 known.
% In this case, pay attention to the fact that also -Th1 can be a valid.
%
noapHst0ip = noap*(Hst0\[pp; 1]);
for i = 1:4:5
    pk2ph = (expScrew([Twist(:,1);Theta_STR4(i,1)]))\noapHst0ip;
    pk2p = pk2ph(1:3);
    w7 = [1 0 0]; x7 = [w7 0 0 0]';
    t7 = PardosGotorThree(x7, [pk2p(1:2); 0], [pg(1:2); 0], norm(pp-pg));
    pk2 = pk2p+w7'*t7(2);
    t51 = PadenKahanOne(Twist(:,5), pp, pk2);
    Theta_STR4(i:i+1,5) = real(t51);
    Theta_STR4(i+2:i+3,5) = real(-t51);
end
%
% STEP3: Calculate Theta6.
% Another geometric formulation for the IK to get t6a (alternative).
ox = noap(1,2); oy = noap(2,2); nx = noap(1,1); ny = noap(2,1);
for i = 1:2:7
    s1 = sin(Theta_STR4(i,1)); c1 = cos(Theta_STR4(i,1));
    s5 = sin(Theta_STR4(i,5));
    t61a = atan2((ox*s1-oy*c1)/s5,(ny*c1-nx*s1)/s5);
    Theta_STR4(i:i+1,6) = real(t61a);
end
%
% STEP4: Calculate Theta2, Theta3, Theta4.
% We pass the exponential of t1 to the right-hand side of the 
% kinematics expression, resulting the formula: 
% E2 * E3 * E4 * E5 * E6 * Hst0 = E1^-1 * Hstt => E2 * E3 * E4 * Hp = Hk
% which is the expression for the PARDOS-GOTOR-EIGHT PG8 canonical problem.
% which has none, one or two triple solutions.
for i = 1:2:7
    Hp = (expScrew([Twist(:,6);Theta_STR4(i,6)]))*Hst0;
    Hp = (expScrew([Twist(:,5);Theta_STR4(i,5)]))*Hp;
    Hk = (expScrew([Twist(:,1);Theta_STR4(i,1)]))\noap;
    t234 = PardosGotorEight(Twist(:,2),Twist(:,3),Twist(:,4),Hp,Hk);
    Theta_STR4(i:i+1,2:4) = t234;
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
    noapi = ForwardKinematicsPOE(TwMagi) * Hst0
end
%
% Check that TcP POSE (rot+tra) as Hst is OK for all Theta values
%