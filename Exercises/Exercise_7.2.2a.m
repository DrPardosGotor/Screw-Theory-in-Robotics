%% Screw Theory - EXAMPLE Trajectory Planning with Inverse Kinematics.
% ABB IRB120 (TOOLDOWN).
% IK Algorithm applied: PG5 + PG4 + PG6 + PK1.
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
%% E721a_ST24R_TP_ABBIRB120_IKPG546PK1_Trapez
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
% Mechanical characteristics of the IRB120 Robot:
po=[0;0;0]; pk=[0; 0; 0.290]; pr=[0; 0; 0.560];
pf=[0.302; 0; 0.630]; pp=[0.302; 0; 0.470];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pk pr pf pf pf];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ AxisY AxisY AxisX AxisY -AxisZ];
Twist = zeros(6,n);
for i = 1:6
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp)*rotY2tform(pi);
%
% Motion RANGE for the robot joints POSITION rad, (by catalog).
Themax = pi/180*[165 110 70 160 120 400];
Themin = -pi/180*[165 110 110 160 120 400];
% Maximum VELOCITY for the robot joints rad/sec, (by catalog).
% Thepmax = pi/180*[250 250 250 320 320 420];
% Thepmin = -pi/180*[250 250 250 320 320 420];
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
    Mag(i,j) = (rand*Themax(j)+rand*Themin(j));
    end
end
%
% Example of Mag for the first exercise in the TP chapter.
%Mag = [   0         0         0         0         0         0;
%    -2.4629   -1.4788   -0.6692   -0.8806   -1.1314   -3.3457;
%    -0.2360    0.0182   -0.0509    1.1307    0.4838   -1.1330;
%     0.9384    0.2239   -0.2360    1.0266   -0.0881    0.0113;
%    -1.6194    0.4954    0.2050   -0.4530   -0.0744   -0.0983;
%     1.4843   -0.2273    0.9737   -0.3518   -0.1730    1.3741;
%     0.7219    1.0669    0.4867   -0.7181    1.4642    1.0728;
%     0.1467    0.2696   -0.3594    0.0637    0.5565    1.9461;
%    -1.6941    0.6255   -0.3665   -0.2611    0.5036    4.5347;
%     0.1590   -0.3994   -1.3927    0.6853   -0.1568    2.2881;
%    -0.4339    0.6387   -0.6289    0.2913   -0.0463   -0.5843];
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
% IK solution approach PG5+PG4+PG6+PK1 subproblems cosecutively.
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
    Theta_STR4 = zeros(8,n);
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
    JointPath(j,:) = Theta_STR4(IkSolution,:);
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
    axis.FontSize = 50;
    plot(traLine, Joint6Trape(i+1,:),'-','LineWidth',5,'DisplayName','q'); hold on;
    plot(traLine, Joint6Trape(i+7,:),'--','LineWidth',5,'DisplayName','qd'); hold on;
    plot(traLine, Joint6Trape(i+13,:),':','LineWidth',5,'DisplayName','qdd'); hold off;
    set(gca,'FontSize',30);
    legend('Pos','Vel','Acc','FontSize',50);
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

