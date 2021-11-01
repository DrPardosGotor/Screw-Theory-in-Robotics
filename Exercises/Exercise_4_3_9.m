%% Screw Theory in Robotics
% An Illustrated and Practicable Introduction to Modern Mechanics
% by CRC Press
% © 2022 Jose M Pardos-Gotor
%
%% Ch4 - INVERSE KINEMATICS.
%
% Exercise 4.3.9: Pardos-Gotor FIVE (PG5).
%
% Calculate IK for a single rotation applied to a LINE or PLANE.
% the movement is defined by the SCREW whose "Twist" parameters
% are defined by: Axis = Axis1, Point = p1, JointType = 'rot'
% and whose magnitude is defined by "Mag".
%
% The movement is aplied to the point pp for moving it to pk.
%
% For checking the working of the PKP1 this exercise has three steps:
% STEP1: Apply ForwardKinemats to the Screw for "whatever" Mag (can be
% even more than 2pi) on pp and then getting a feasible pk.
% STEP2: Calculate the IK solution by PKP1 getting the magnitud Theta1
% STEP3: Test the PKP1 solution applying ForwardKinemats to the Screw with
% Theta1 on pp and checking we get the same pk.
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
%% MATLAB Code
%
clear
clc
%
pp = [rand*10 rand*10 rand*10]'
Mag = (rand-rand)*2*pi
%
Axis1 = [0 1 0]';
p1 = [0 0 0]';
JointType1 = 'rot';
Twist = joint2twist(Axis1, p1, JointType1);
%
% STEP1: Apply ForwardKinemats to the Screw for "whatever" Mag
TwMag1 = [Twist; Mag];
HstR1 = ForwardKinematicsPOE(TwMag1);
pk1h = HstR1*[pp; 1];
pk1 = pk1h(1:3)
%
% STEP2: Calculate the IK solution by PK1 getting the magnitud Theta11
Theta1 = PardosGotorFive(Twist, pp, pk1)
%
% STEP3: Test the PKP1 solution applying ForwardKinemats to the Screw
TwMag2 = [Twist; Theta1(1)];
HstR2 = ForwardKinematicsPOE(TwMag2);
pk2h = HstR2*[pp; 1];
pk2 = pk2h(1:3)
%
Ang = Theta1(1)-Theta1(2);
% Check that (pk1 = pk2) 
%