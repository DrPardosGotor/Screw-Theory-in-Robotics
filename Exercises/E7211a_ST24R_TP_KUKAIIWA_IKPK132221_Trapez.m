%% Screw Theory - EXAMPLE Trajectory Planning with Inverse Kinematics.
% KUKA IIWA 14.
% IK Algorithm applied: PK1 + PK3 + PK2 + PK2 + PK2 + PK1.
% Trapezoidal interpolation for Joint Trajectory Planning.
%
% The goal of this exercise is to TEST:
% TRAJECTORY PLANNING
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% For checking the quality of this IK solution, this exercise has 5 steps:
% STEP1: Apply ForwardKinemats for the Robot for random Mag Theta1...7
% getting a feasible set of TcP configuration (rot+tra) PATH in task-space.
% STEP2: Calculate the IK solutions by SCREW THEORY management getting
% the magnitud Theta1...7. There can be up to 16 right solutions.
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
%% E7211a_ST24R_TP_KUKAIIWA_IKPK132221_Trapez
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
% n is number of DoF, this is the number of Joints.
DoF = 7;
%
% Mechanical characteristics of the Robot:
po=[0;0;0]; pk=[0;0;0.36]; pr=[0;0;0.78];
pf=[0;0;1.18]; pp=[0;0;1.38];
%AxisX = [1 0 0]';
AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pk po pr po pf po];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ AxisY AxisZ -AxisY AxisZ AxisY AxisZ];
Twist = zeros(6,7);
for i = 1:7
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp);
%
% Motion RANGE for the robot joints POSITION rad, (by catalog).
Themax = pi/180*[170 120 170 120 170 120 175];
Themin = -pi/180*[170 120 170 120 170 120 175];
% Maximum SPEED for the robot joints rad/sec, (by catalog).
Thepmax = pi/180*[85 85 100 75 130 135 135];
Thepmin = -pi/180*[85 85 100 75 130 135 135];
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
Mag = zeros(TcpNumWayPoints+1,DoF);
for i = 2:TcpNumWayPoints+1
    for j = 1:DoF
    Mag(i,j) = 1/2*(rand*Themax(j)+rand*Themin(j));
    end
end
%
% Example of Mag for the first exercise in the TP chapter.
Mag = [  0         0         0         0         0         0         0;
   -0.1144   -0.1056   -0.2171    0.0364    0.0357   -0.0337   -0.1252;
   -0.0459   -0.1965    0.2483   -0.0066   -0.2124    0.0061    0.3936;
   -0.0478   -0.1008    0.0866   -0.1114   -0.1136   -0.2519   -0.1664;
    0.1748   -0.1935   -0.1943   -0.0644    0.2323   -0.1746    0.0623;
   -0.0559    0.0079   -0.2102    0.0927   -0.2111   -0.0690    0.0927;
   -0.2035   -0.1335   -0.2464    0.0995    0.3092   -0.0621   -0.2772;
    0.2692    0.1372   -0.2710    0.0152    0.0791    0.0701   -0.0378;
   -0.0471   -0.0494    0.2584   -0.0135    0.0600   -0.1143   -0.1514;
   -0.0415    0.1818    0.0361    0.2122    0.1577    0.0837   -0.2536;
    0.0524    0.1353    0.1375   -0.2527   -0.2737    0.1290    0.0021];
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
TcPTra = zeros(DoF+1,traSize);
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
% IK solution approach PK1 + PK3 + PK2 + PK2 + PK2 + PK1 subproblems.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IK chosen solution. It could be whatever value from 1 to 16.
IkSolution = 9;
%
% Joint path rows are the path points.
% Joint path columns are the Joint 1..n positions for each point.
JointPath = zeros(TcpNumWayPoints+1,DoF);
%
for j = 2:TcpNumWayPoints+1
    % The input to the IK algorithm is the desired TcP TARGET in terms of
    % homegeneous matrix with info for TcP position and orientation
    noap = [eul2rotm(TcpPath(j,4:6),'XYZ') TcpPath(j,1:3)'; 0 0 0 1];
    %
    % Matrix to save all possible IK solutions.
    % columns are solutions and rows Joint 1..DoF values for each solution.
    Theta = zeros(16,DoF);
    %
    % First, as this is REDUNDANT robot with 7DoF and theoretical infinite
    % solutions, we reduce the number of possibilies to 16, and for doing so
    % we give some solution INPUTS for t1 and t3 based on some other criteria,
    % and then well get 8 solutions for each of these INPUTS for the rest 6DoF.
    % For this function we choose t1 and t3 to be the magnitudes for orientate
    % these joint towards the TcP, and for doing so we use PADEN-KAHAN-ONE.
    t1 = PadenKahanOne(Twist(:,1), [1; 0; 0], noap(1:3,4));
    t3 = PadenKahanOne(Twist(:,3), [1; 0; 0], noap(1:3,4));
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
        Theta(i:i+1,5:6) = PadenKahanTwo(Twist(:,5),Twist(:,6),pp,pk3p); 
    end
    %
    % STEP2-5: Calculate Theta7.
    % With "[1; 0; 0]" not in the axis of E7 apply E6^-1...*E1^-1*noap*gs0^-1 to "po"
    % and applying E7 to "po" knowing already Theta6...Theta1,
    % resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
    % exp(E7^theta7)*po = pk4p ; with
    % pk4p = exp(E6^Th6)^-1*...*exp(E1^Th1)^-1*noap*hst0^-1*po 
    % which by PADEN-KAHAN-ONE has none or one solution. Then for all
    % Th6-Th5-Th4-Th3-Th2-Th1 we get a Th7 = t701...t716:
    %
    noapHst0io = noap*(Hst0\[1; 0; 0; 1]);
    for i = 1:16
        pk4pt = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0io;
        pk4pt = (expScrew([Twist(:,2);Theta(i,2)]))\pk4pt;
        pk4pt = (expScrew([Twist(:,3);Theta(i,3)]))\pk4pt;
        pk4pt = (expScrew([Twist(:,4);Theta(i,4)]))\pk4pt;
        pk4pt = (expScrew([Twist(:,5);Theta(i,5)]))\pk4pt;
        pk4pt = (expScrew([Twist(:,6);Theta(i,6)]))\pk4pt;
        pk4p = pk4pt(1:3);
        Theta(i,7) = PadenKahanOne(Twist(:,7), [1; 0; 0], pk4p);
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
% rows 2-8 is Joint Theta Positon q (rad).
% rows 9-15 is Joint Theta Velocity qd (rad/s).
% rows 16-22 is Joint Theta Acceleration qdd (rad/ss).
Joint7Trape = zeros((DoF*3)+1,traSize);
Joint7Trape(1,:) = traLine;
%
for i = 1:DoF
    [q, qd, qdd] = trapveltraj([pathLine; JointPath(:,i)'], traSize);
    Joint7Trape(i+1,:) = q(2,:);
    Joint7Trape(i+8,:) = qd(2,:);
    Joint7Trape(i+15,:) = qdd(2,:);
    figure(i);
    plot(traLine, Joint7Trape(i+1,:),'LineWidth',2,'DisplayName','q'); hold on;
    plot(traLine, Joint7Trape(i+8,:),'LineWidth',2,'DisplayName','qd'); hold on;
    plot(traLine, Joint7Trape(i+15,:),'LineWidth',2,'DisplayName','qdd'); hold off;
end
%
% Create a MATLAB file with the Joint TRAJECTORY.
ans = Joint7Trape;
save( 'Traje7RTrape','ans','-v7.3');
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
    TwMag = [Twist; Joint7Trape(2:(DoF+1),i)'];
    HstR = ForwardKinematicsPOE(TwMag) * Hst0;
    TcPTra(2:7,i) = [HstR(1:3,4)' rotm2eul(HstR(1:3,1:3), 'XYZ')]';
end
% Create a MATLAB file with the TcP RESULT TRAJECTORY.
ans = TcPTra;
save( 'ToolVAL','ans','-v7.3');
%

