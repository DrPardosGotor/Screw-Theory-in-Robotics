%% Screw Theory in Robotics
% An Illustrated and Practicable Introduction to Modern Mechanics
% by CRC Press
% © 2022 Jose M Pardos-Gotor
%
%% Ch4 - INVERSE KINEMATICS.
%
% Exercise 4.3.10: Pardos-Gotor SEVEN (PG7).
%
% Calculate IK for three consecutive rotation SCREWS.
% Find the Inverse Kinematics of 3 consecutive ROTATIONS SCREWS with
% two PARALLEL axes and one SKEW axis.
% which applied to a point move it ot a different point in space in SE(3)
%
% the movements are defined by the SCREWS whose "Twists" parameters
% are: Axis = [Axis1 Axis2 Axis3], Point = [p1 p2 p3], JointType = 'rot'
% and whose magnitudes are defined by Mag = [Theta1 Theta2 Theta3].
%
% Compute angles "Th3", "Th2" & "Th1" of three subsequently applies SCREWS
% with corresponding twists x3, x2 and x1, to move the point "p" to "k".
% Beware of the order for the movement: first X3 + x2 & subsequently x1.
% Point p is first moved to the point "e" or "f" by the Screw with Theta3
% then moved to the point "c" or "d" by the Screw with Theta2
% and then moved from "c" or "d" to the point "k" by the Screw with Theta1.
%
% For checking the this function, this exercise has 3 steps:
% STEP1: Apply ForwardKinemats to the Screws for any Mag (t3 + t2 + t1)
% (can be even more than 2pi) on pp and then getting a feasible pk.
% STEP2: Calculate the IK solution by PG6 getting the magnitud
% The problem could have up to FOUR TRIPLE solutions
% Theta123 = [t1ck t2ec t3pe; t1ck t2fc t3pf; t1dk t2ed t3pe; t1dk t2fd t3pf;]
% as a consequence of paths: p-e-c-k, p-f-c-k, p-e-d-k, p-f-d-k.
% STEP3: Test the solutions applying
% ForwardKinemats to the Screws on pp and checking we get the same pk.
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
% You should have received a copy of the GNU Leser General Public License
% along with ST24R. If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2021/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% MATLAB Code
%
clear
clc
%
pp = [rand*10 rand*10 rand*10]' % for testing various initial points
%pp = [0 0 6]'
Mag = [(rand-rand)*2*pi (rand-rand)*2*pi (rand-rand)*2*pi] % for testing various magnitudes
%Mag = [pi/2 pi]
%
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]';
%
r1 = [0 0 0]'; r2 = [1 0 1]'; r3 = [1 0 2]';% points for the Screw Axes.
Point = [r1 r2 r3];
% Axis for w1 must be skew and w2 and w3 parallel.
Axis = [AxisZ AxisY AxisY];
JointType = ['rot'; 'rot'; 'rot']; % whatever for testing the exercise
% 
% Now we build the TWISTS matrix for the chosen Joints
Twist = joint2twist(Axis(:,1), Point(:,1), JointType(1,:));
for i = 2:size(Point,2)
    Twist = [Twist joint2twist(Axis(:,i), Point(:,i), JointType(i,:))];
end
%
% STEP1: Apply ForwardKinemats to the TWO Screws x2 and then x1 on pp for
% "whatever" Mag (can be% even more than 2pi) for getting a feasible pk.
TwMag1 = [Twist; Mag];
HstR1 = ForwardKinematicsPOE(TwMag1);
pk1h = HstR1*[pp; 1];
pk1 = pk1h(1:3)
% STEP2: Calculate the IK solution by PK2 getting the magnitud
% Theta1Theta2 = [t11 t21; t12 t22] DOUBLE SOLUTION.
The123 = PardosGotorSeven(Twist(:,1), Twist(:,2), Twist(:,3), pp, pk1)
%
% STEP3: Test the solutions applying ForwardKinemats to the Screws
% on pp and checking we get the same pk.
for i = 1:size(The123,1)
    TwMagi = [Twist; The123(i,:)];
    HstRi = ForwardKinematicsPOE(TwMagi);
    i
    pki = HstRi*[pp; 1];
    pk = pki(1:3)
end
%
% Check that pki = pk) 
%