%% Screw Theory - EXAMPLE Trajectory Planning with Inverse Kinematics.
% UR16e.
% IK Algorithm applied: PG5 + PG3 + PK1 + PG8.
% Trapezoidal interpolation for Joint Trajectory Planning.
%
% The goal of this exercise is to TEST:
% TRAJECTORY PLANNING
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
%% E729a_ST24R_TP_UR16e_IKPG53PK1PG8_Trapez
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
Hst0 = trvP2tform(pp)*rotX2tform(-pi/2)*rotZ2tform(pi);%
%
% Motion RANGE for the robot joints POSITION rad, (by catalog).
Themax = pi/180*[360 360 360 360 360 360];
Themin = -pi/180*[360 360 360 360 360 360];
% Maximum SPEED for the robot joints rad/sec, (by catalog).
%Thepmax = pi/180*[120 120 180 180 180 180];
%Thepmin = -pi/180*[120 120 180 180 180 180];
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
    -0.2041    0.0862   -0.0735    0.4137   -0.4760   -0.4562;
     0.1762   -0.6435   -0.4170   -0.4217   -0.6076   -0.0008;
    -0.2540    0.3415    0.4313   -0.2339    0.5598   -0.1300;
     0.1416   -0.0130    0.0827   -0.2367    0.1837    0.6287;
    -0.1643    0.1010    0.5229    0.5780    0.4982   -0.3050;
    -0.4231    0.5976    0.2253   -0.2410    0.5448    0.1624;
     0.2432    0.3727   -0.0159   -0.2887   -0.3264    0.0025;
     0.5483   -0.5624    0.0359    0.7122    0.0177   -0.7743;
    -0.0373    0.1737    0.1277    0.5477   -0.5515    0.2641;
    -0.2954    0.6055   -0.1669    0.6631   -0.1718   -0.4071];
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
% IK solution approach PG5+PG3+PK1+PG8 subproblems cosecutively.
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

