%% Screw Theory - EXAMPLE Trajectory Planning with Inverse Kinematics.
% ABB IRB1600 (TOOLDOWN).
% IK Algorithm applied: PG7 + PG6 + PK1.
% Trapezoidal interpolation for Joint Trajectory Planning.
%
% The goal of this exercise is to TEST:
% TRAJECTORY PLANNING for IRB120 with ToolDown POSE
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% For checking the quality of this IK solution, this exercise has 5 steps:
% STEP1: Apply ForwardKinemats for the Robot for random Mag Theta1...6
% getting a feasible set of TcP configuration (rot+tra) PATH in task-space.
% STEP2: Calculate the IK solutions by SCREW THEORY management getting
% the magnitud Theta1...6. There can be up to 8 right solutions for this
% problem using this approach (theoretically there is max of 16 solutions).
% Only one solution is chosen to build a set of Theta PATH in joint-space.
% STEP3: Test the joint path plannig applying Forward Kinemats only the 
% Waypoints of interest in the PATH checking TcP congiguration (rot+tra).
% STEP4: interpolate the joint path plannig with to complete a Joint
% Trajectory for the whole trajectory line.
% STEP5: Test the joint trajectory plannig applying Forward Kinemats to all
% the points in the TRAJECTORY checking TcP congiguration (rot+tra).
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
%% E723a_ST24R_TP_ABBIRB1600_IKPG76PK1_Trapez
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
n = 6;
%
% Mechanical characteristics of the Robot:
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
%
% Motion RANGE for the robot joints POSITION rad, (by catalog).
Themax = pi/180*[180 136 55 200 115 400];
Themin = -pi/180*[180 63 235 200 115 400];
% Maximum SPEED for the robot joints rad/sec, (by catalog).
%Thepmax = pi/180*[150 160 170 320 400 460];
%Thepmin = -pi/180*[150 160 170 320 400 460];
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
Mag = [   0         0         0         0         0         0;
     1.1055    0.7524   -1.1112   -0.7451   -0.1266   -1.4154;
    -0.3571    0.2453   -1.1633    0.8044   -0.5322   -1.7740;
     0.3084   -0.3768   -1.4818   -0.8199   -0.6793   -0.0031;
    -0.4445    0.7698    0.0931   -0.4549    0.6259   -0.5055;
     0.2478    0.4699   -1.3437   -0.4602    0.2054    2.4450;
    -0.2876    0.3124   -0.2010    1.1239    0.5570   -1.1862;
    -0.7404    0.9537   -0.3936   -0.4686    0.6091    0.6315;
     0.4256    0.4933   -0.8949   -0.5613   -0.3649    0.0098;
     0.9595   -0.1295   -0.1434    1.3848    0.0198   -3.0113;
    -0.0653    0.4161   -0.8336    1.0650   -0.6166    1.0270];
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
% IK solution approach PG7+PG6+PK1 subproblems cosecutively.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IK chosen solution. It could be whatever value from 1 to 8.
IkSolution = 5;
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
    Theta_STR6 = zeros(8,n);
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
    JointPath(j,:) = Theta_STR6(IkSolution,:);
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
% rows 2-7 is Joint Theta Positon q (rad).
% rows 8-13 is Joint Theta Velocity qd (rad/s).
% rows 14-19 is Joint Theta Acceleration qdd (rad/ss).
Joint6Trape = zeros(19,traSize);
Joint6Trape(1,:) = traLine;
%
for i = 1:n
    [q, qd, qdd] = trapveltraj([pathLine; JointPath(:,i)'], traSize);
    Joint6Trape(i+1,:) = q(2,:);
    Joint6Trape(i+7,:) = qd(2,:);
    Joint6Trape(i+13,:) = qdd(2,:);
    figure(i);
    plot(traLine, Joint6Trape(i+1,:),'LineWidth',2,'DisplayName','q'); hold on;
    plot(traLine, Joint6Trape(i+7,:),'LineWidth',2,'DisplayName','qd'); hold on;
    plot(traLine, Joint6Trape(i+13,:),'LineWidth',2,'DisplayName','qdd'); hold off;
end
%
% Create a MATLAB file with the Joint TRAJECTORY.
ans = Joint6Trape;
save( 'Traje6RTrape','ans','-v7.3');
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
    TwMag = [Twist; Joint6Trape(2:7,i)'];
    HstR = ForwardKinematicsPOE(TwMag) * Hst0;
    TcPTra(2:7,i) = [HstR(1:3,4)' rotm2eul(HstR(1:3,1:3), 'XYZ')]';
end
% Create a MATLAB file with the TcP RESULT TRAJECTORY.
ans = TcPTra;
save( 'ToolVAL','ans','-v7.3');
%

