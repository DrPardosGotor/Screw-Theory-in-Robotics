%% Screw Theory in Robotics
% An Illustrated and Practicable Introduction to Modern Mechanics
% by CRC Press
% © 2022 Jose M Pardos-Gotor
%
%% Ch7 - TRAJECTORY GENERATION.
%
% Exercise 7.2.6a: ABB IRB910SC.
% IK Algorithm applied: PG1 + PG4 + PK1.
% Trapezoidal interpolation for Joint Trajectory Planning.
%
% The goal of this exercise is to TEST:
% TRAJECTORY PLANNING
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% For checking the quality of this IK solution, this exercise has 5 steps:
% STEP1: Apply ForwardKinemats for the Robot for random Mag Theta1...4
% getting a feasible set of TcP configuration (rot+tra) PATH in task-space.
% STEP2: Calculate the IK solutions by SCREW THEORY management getting
% the magnitud Theta1...4. There can be up to 2 right solutions
% Only one solution is chosen to build a set of Theta PATH in joint-space.
% STEP3: Test the joint path plannig applying Forward Kinemats only the 
% Waypoints of interest in the PATH checking TcP congiguration (rot+tra).
% STEP4: interpolate the joint path plannig with to complete a Joint
% Trajectory for the whole trajectory line.
% STEP5: Test the joint trajectory plannig applying Forward Kinemats to all
% the points in the TRAJECTORY checking TcP congiguration (rot+tra).
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
%% MATLAB Code.
%
clear
clc
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% characteristics for Tool TcP PATH and TRAJECTORY planning in
% task-space (feasible).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% number of wayPoints in TcP path planning, it includes Goal Point but not
% Start Point, which is for the robot home pose with Joint Theta = ZERO.
TcpNumWayPoints = 10;
pathLine = (0:1:TcpNumWayPoints);
%
% TcP trajectory planning timing sec.
traTime = 10;
% TcP trajectory planning stamp sec.
traSample = 0.001;
traLine = (0:traSample:traTime);
traSize = size(traLine,2);
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mechanical characteristics of the Robot (AT REF HOME POSITION):
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% n is number of DOF.
n = 4;
%
% Mechanical characteristics of the Robot:
po=[0;0;0]; pr=[0.4;0;0]; pf=[0.65;0;0]; pp=[0.65;0.125;0]; 
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pr pf pp];
Joint = ['rot'; 'rot'; 'tra'; 'rot'];
Axis = [AxisY AxisY AxisY -AxisY];
Twist = zeros(6,n);
for i = 1:4
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp)*rotX2tform(pi/2)*rotZ2tform(pi);
%
%
%Motion RANGE for the robot joints POSITION rad, (by catalog).
Themax = [pi/180*140 pi/180*150 0 pi/180*400];
Themin = [-pi/180*140 -pi/180*150 -0.125 -pi/180*400];
% Maximum SPEED for the robot joints m/s and rad/sec, (by catalog).
%Thepmax = [7.58 7.58 1.02 pi/180*2400];
%Thepmin = -[7.58 7.58 1.02 pi/180*2400];
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tool TcP PATH planning in task-space (feasible)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TcP path rows are the path points.
% TcP path columns are the position and orientation (X-Y-Z) por each point.
TcpPath = zeros(TcpNumWayPoints+1,6);
%
% Random Joint Theta magnitudes to generate feaseble TcP Targets by FK.
% Mag rows are the path point.
% Mag columns are the Joint 1..n positions for each point.
Mag = zeros(TcpNumWayPoints+1,n);
for i = 2:TcpNumWayPoints+1
    for j = 1:n
    Mag(i,j) = 1/2*(rand*Themax(j)+rand*Themin(j));
    end
end
%
% Example of Mag for the first exercise in the TP chapter.
Mag = [   0         0         0         0;
     0.3545   -0.4046   -0.0438    0.0346;
    -0.0011    0.8708   -0.1198    0.0751;
    -0.1320   -0.2955   -0.0738    0.0043;
    -0.1955    0.7798   -0.0171    0.0101;
     0.3482    0.5865   -0.0271    0.0610;
     0.2614    0.1384   -0.0206   -0.0555;
    -0.1475    0.1261   -0.0453    0.0425;
    -0.0897   -0.4351   -0.0744    0.0257;
    -0.0612    0.6569   -0.1079    0.0611;
     0.0782    0.9072   -0.1112   -0.0089];
%
% Forward Kinemats to get the Tool set of TARGETS, which is the TcP PATH.
for i = 1:TcpNumWayPoints+1
    TwMag = [Twist; Mag(i,:)];
    noap = ForwardKinematicsPOE(TwMag)*Hst0;
    TcpPath(i,:) = [noap(1:3,4)' rotm2eul(noap(1:3,1:3), 'XYZ')];
end
%
% Create a MATLAB file with the TcP REFERENCE TRAJECTORY.
% row 1 is the trajectory time line.
% rows 2-4 are the TcP position X-Y-Z m.
% rows 5-7 are the TcP orientation with Euler X-Y-Z rad.
TcPTra = zeros(7,traSize);
TcPTra(1,:) = traLine;
for i = 1:6
    TcPTra(i+1,:) = interp1(pathLine, TcpPath(:,i)',traLine);
end
ans = TcPTra;
save( 'ToolREF','ans','-v7.3');
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint PATH planning in joint-space with Inverse Kinematics algorithm.
% Calculate the IK solutions Theta using the SCREW THEORY
% IK solution approach PG1+PG4+PK1 subproblems cosecutively.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IK chosen solution. It could be whatever value from 1 to 4.
IkSolution = 1;
%
% Joint path rows are the path points.
% Joint path columns are the Joint 1..n positions for each point.
JointPath = zeros(TcpNumWayPoints+1,n);
%
for j = 2:TcpNumWayPoints+1
    % The input to the IK algorithm is the desired TcP TARGET in terms of
    % homegeneous matrix with info for TcP position and orientation
    noap = [eul2rotm(TcpPath(j,4:6),'XYZ') TcpPath(j,1:3)'; 0 0 0 1];
    %
    % Matrix to save all possible IK solutions.
    % columns are solutions and rows Joint 1..n values for each solution.
    Theta = zeros(2,n);
    %
    % STEP2: Calculate Theta3.
    % With "pf" on the axis of E4. We apply (noap*hs0^-1) to "pf"
    % doing so we get Theta3 applying the Canonic problem PARDOS-ONE,
    % because the screws E4 do not affect "pf" for being on its axis
    % and the E1,E2 do not change the plane where "pf" moves (perpendicular to
    % the axis of those screws, and so do not affect the calculation for Theta3
    % resulting the problem "exp(E3^theta3)*pf = noap*hst0^-1*pf" by PARDOS-ONE
    % which has one solution for t31.
    pkp = noap*(Hst0\[pf; 1]);
    Theta(1,3) = PardosGotorOne(Twist(:,3), pf, pkp(1:3));
    % prepare Theta for next calculation
    Theta(2,3) = Theta(1,3);
    %
    % STEP2: Calculate Theta1 & Theta2.
    % With "pf" on the axis of E4, we apply (noap*hs0^-1) to "pf" and
    % the POE E1..E4 also to "pf" having already known the value for Theta3
    % resulting exactly a Canonic problem PARDOS-FOUR, because the screw
    % E4 do not affect "pf" and the E3 is known,resulting the problem
    % exp(E1^theta1)*exp(E2^theta2)*exp(E3^theta3)*pf = 
    % exp(E1^theta1)*exp(E2^theta2)*pfp = noap*hst0^-1*pf = pkp
    % which by PARDOS-FOUR has none, one or two DOUBLE solutions.
    % t11-t21 & t12-t22 for each value of t31
    %
    pfp = expScrew([Twist(:,3);Theta(1,3)])*[pf; 1];
    Theta(1:2,1:2) = PardosGotorFour(Twist(:,1),Twist(:,2),pfp(1:3),pkp(1:3));
    %
    % STEP2: Calculate Theta4.
    % With "po" not in the axis of E4 apply E3^-1...*E1^-1*noap*hst0^-1 to "po"
    % and applying E4 to "po" knowing already Theta3...Theta1 solutions,
    % resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
    % exp(E4^theta4)*po = pk3p ; with
    % pk3p = exp(E3^Th3)^-1*...*exp(E1^Th1)^-1*noap*hst0^-1*po 
    % which by PADEN-KAHAN-ONE has none or one solution. Then for all
    % Th3-Th2-Th1 known (two solutions) we get t4:
    noapHst0io = noap*(Hst0\[po; 1]);
    for i = 1:2
        pk3p = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0io;
        pk3p = (expScrew([Twist(:,2);Theta(i,2)]))\pk3p;
        pk3p = (expScrew([Twist(:,3);Theta(i,3)]))\pk3p;
        Theta(i,4) = PadenKahanOne(Twist(:,4), po, pk3p(1:3));
    end
    %    
    %
    JointPath(j,:) = Theta(IkSolution,:);
    %
end
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test the quality of the Joint Path obtained
% Apply Forward kinematics to get the real TcP RESULT PATH
% Thist test the results only for the Waypoints of interest
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TcP Value path rows are the path points.
% TcP Value path columns are position and orientation (X-Y-Z) each point.
TcpVal = zeros(TcpNumWayPoints+1,6);
%
for i = 1:TcpNumWayPoints+1
    TwMag = [Twist; JointPath(i,:)];
    HstR = ForwardKinematicsPOE(TwMag) * Hst0;
    TcpVal(i,:) = [HstR(1:3,4)' rotm2eul(HstR(1:3,1:3), 'XYZ')];
end
%
% Show both paths, the TARGET TcPPath and the real VALUE TcPVal
TcpPath
%JointPath
TcpVal
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint TRAJECTORY - TRAPEZOIDAL motion.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% row 1 is the trajectory line time.
% rows 2-5 is Joint Theta Positon q (rad).
% rows 6-9 is Joint Theta Velocity qd (rad/s).
% rows 10-13 is Joint Theta Acceleration qdd (rad/ss).
Joint6Trape = zeros(13,traSize);
Joint6Trape(1,:) = traLine;
%
for i = 1:n
    [q, qd, qdd] = trapveltraj([pathLine; JointPath(:,i)'], traSize);
    Joint6Trape(i+1,:) = q(2,:);
    Joint6Trape(i+5,:) = qd(2,:);
    Joint6Trape(i+9,:) = qdd(2,:);
    figure(i);
    plot(traLine, Joint6Trape(i+1,:),'LineWidth',2,'DisplayName','q'); hold on;
    plot(traLine, Joint6Trape(i+5,:),'LineWidth',2,'DisplayName','qd'); hold on;
    plot(traLine, Joint6Trape(i+9,:),'LineWidth',2,'DisplayName','qdd'); hold off;
end
%
% Create a MATLAB file with the Joint TRAJECTORY.
ans = Joint6Trape;
save( 'Traje4Trape','ans','-v7.3');
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test the quality of the Joint Trajectory obtained
% Apply Forward kinematics to get the real TcP RESULT TRAJECTORY 
% Thist test the results for all points in the trajectory line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% row 1 is the trajectory time line.
% rows 2-4 are the TcP position X-Y-Z m.
% rows 5-7 are the TcP orientation with Euler X-Y-Z rad.
TcPTra = zeros(7,traSize);
TcPTra(1,:) = traLine;
for i = 1:traSize
    TwMag = [Twist; Joint6Trape(2:5,i)'];
    HstR = ForwardKinematicsPOE(TwMag) * Hst0;
    TcPTra(2:7,i) = [HstR(1:3,4)' rotm2eul(HstR(1:3,1:3), 'XYZ')]';
end
% Create a MATLAB file with the TcP RESULT TRAJECTORY.
ans = TcPTra;
save( 'ToolVAL','ans','-v7.3');
%

