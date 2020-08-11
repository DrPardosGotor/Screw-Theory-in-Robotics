%% Screw Theory - CANONICAL Inverse Kinematics.
% Pardos-Gotor EIGHT (PG8).
%
% Calculate IK for three rotation SCREWS which are applied to a POSE.
% Find the Inverse Kinematics of 3 consecutive SCREW ROTATIONS
% SCREWS with PARALLEL axes.
% which applied to a configuratio POSE (postion + orientation)
% move it ot a different POSE configuration in space in SE(3).
%
% the movements are defined by the SCREWS whose "Twists" parameters
% are: Axis = [Axis1 Axis2 Axis3], Point = [p1 p2 p3], JointType = 'rot'
% and whose magnitudes are defined by Mag = [Theta1 Theta2 Theta3].
%
% Compute angles "Th3", "Th2" & "Th1" of three subsequently applies SCREWS
% with corresponding twists x3, x2 and x1, to move the pose "Hp" to "Hk".
% Beware of the order for the movement: first X3 + x2 & subsequently x1.
%
% For checking the this function, this exercise has 3 steps:
% STEP1: Apply ForwardKinemats to the Screws for any Mag (t3 + t2 + t1)
% (can be even more than 2pi) on Hp and then getting a feasible Hk.
% STEP2: Calculate the IK solution by PG8 getting the magnitud
% The problem could have up to TWO TRIPLE solutions
% Theta123 = [t1co t2oc t31; t1do t2od t32]
% STEP3: Test the solutions applying
% ForwardKinemats to the Screws on Hp and checking we get the same Hk.
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
% along with ST24R. If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2020/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% E4311_STR24R_CIK_PardosGotor_EIGHT
%
clear
clc
%
% Magnitudes for the screws t1-t2-t3.
Mag = [(rand-rand)*pi (rand-rand)*pi (rand-rand)*pi]
%
% for testing various initial pose Hp.
Rp = eul2rotm([(rand-rand)*pi (rand-rand)*pi (rand-rand)*pi], 'ZYX');
Pp = [(rand-rand) (rand-rand) (rand-rand)];
Hp = [Rp Pp'; 0 0 0 1]
%
% points for the Screw Axes.
r1 = [5 0 0]';
r2 = [3 0 0]';
r3 = [1 0 0]';
Point = [r1 r2 r3];
% Axis for w1 must be skew and w2 and w3 parallel.
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]';
Axis = [AxisY AxisY AxisY];
JointType = ['rot'; 'rot'; 'rot']; % whatever for testing the exercise
% 
% Now we build the TWISTS matrix for the chosen Joints
Twist = joint2twist(Axis(:,1), Point(:,1), JointType(1,:));
for i = 2:size(Point,2)
    Twist = [Twist joint2twist(Axis(:,i), Point(:,i), JointType(i,:))];
end
%
% STEP1: Apply ForwardKinematics to the TWO Screws x2 and then x1 on pp for
% "whatever" Mag (can be% even more than 2pi) for getting a feasible pk.
TwMag1 = [Twist; Mag];
Hk = ForwardKinematicsPOE(TwMag1)*Hp
%
% STEP2: Calculate the IK solution.
t123 = PardosGotorEight(Twist(:,1), Twist(:,2), Twist(:,3), Hp, Hk)
%
% STEP3: Test the solutions applying ForwardKinemats to the Screws
% on Hp and checking we get the same configuration of Hk.
for i = 1:size(t123,1)
    TwMagi = [Twist; t123(i,:)];
    Hki = ForwardKinematicsPOE(TwMagi)*Hp
end
%
% Check that pki = pk) 
%