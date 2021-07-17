%% Screw Theory in Robotics
% An Illustrated and Practicable Introduction to Modern Mechanics
% by CRC Press
% © 2022 Jose M Pardos-Gotor
%
%% Ch7 - TRAJECTORY GENERATION.
%
% Exercise 7.2.8b: KUKA IIWA 14.
% Invese Differential Kinematics Algorithm with Screw Geometric Jacobian.
% Quintic Interpolation for the Joint Path Planning.
%
% The goal of this exercise is to TEST:
% TRAJECTORY PLANNING inside the workspace of the robot.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% For checking the quality of this IK solution, this exercise has 5 steps:
% STEP1: Apply ForwardKinemats for the Robot for random Mag Theta1...7
% getting a feasible set of TcP configuration (rot+tra) PATH in task-space.
% Employ some interpolation method to complete the trajectory between the
% points of the end-effector path planning, till the discretization of
% timeline. Small enough for limited changes of path planning magnitudes
% between consecutive points.
% STEP2: Evaluate the inverse DK by SCREW THEORY getting an approximation
% to the magnitud Theta1...7. Only one solution in joint-space.
% the INVERSE (Joint Thetap Velocities) DK, based on Tool Velocities.
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
% n is number of DoF, this is number of Joints.
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
    Mag(i,j) = 1/7*(rand*Themax(j)+rand*Themin(j));
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
TcPTra = zeros(7,traSize);
TcPTra(1,:) = traLine;
for i = 1:6
    TcPTra(i+1,:) = interp1(pathLine, TcpPath(:,i)',traLine, 'v5cubic');
end
ans = TcPTra;
save( 'ToolREF','ans','-v7.3');
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint PATH planning in joint-space with Inverse DIFFEREENTIAL KINEMATICS.
% Solutions for Theta by using the SCREW THEORY with Geometric Jacobiaan
% and the integration of the joint velocities.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint path rows are the path points.
% Joint path columns are the Joint 1..n positions for each point.
JointPath = zeros(traSize,DoF);
%
for i = 1:traSize-1
    % First we calculate the Tool end-effector velocities at current pose.
    % as the difference between actual and next Tool pose configurations
    % Both expressed as a Tool pose [trvX trvY trvZ rotX rotY rotZ] in
    % Cartesian adn Euler coordinates with scheme X-Y-Z.
    % VtS is the classical velocity for Tool Pose in spatial frame (S).
    % consider the differentiation step size.
    TargetVAL = TcPTra(2:7,i)';
    TargetREF = TcPTra(2:7,i+1)'; 
    VtS = minusposeEul(TargetVAL, TargetREF) / traSample;
    %
    % GEOMETRIC JACOBIAN JstS and SPATIAL TWIST VELOCITY "VstS" 
    % are defined at current trajectory pose.
    JstS = GeoJacobianS([Twist; JointPath(i,:)]);
    VstS = [VtS(1:3)-axis2skew(VtS(4:6))*TargetVAL(1:3)'; VtS(4:6)];
    %
    % Next formulation is slower, but works too for Non-Squarre matrices.
    %Thetap = (pinv(JstS)*VstS)'; % it is giving worse results.
    % This formulation is faster, but only works for Square matrices 
    %Thetap = (JstS\VstS)';
    % Using Moore-Penrose generalized inverse for getting Theta velocities.
    Thetap = ((pinv(JstS'*JstS) * JstS')*VstS)';
    % The Theta VELOCITIES values are limited by the joints spped limits.
    Thetap = jointmag2limits(Thetap, Thepmax, Thepmin);
    %
    % from Inverse DK we get the incremental joint coordinates
    % and then integrating with EULER Explicit Method the Theta VALUE 
    % with the new joint positions vector OUTPUT.
    % consider the integration step size.
    Theta = JointPath(i,:) + (Thetap * traSample);
    % The Theta POSITION values are limited by the joints position limits.
    Theta = jointmag2limits(Theta, Themax, Themin);
    %
    JointPath(i+1,:) = Theta;
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
    TwMag = [Twist; JointPath(((i-1)/traSample)+1,:)];
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
% Joint TRAJECTORY - QUINTIC motion.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% row 1 is the trajectory line time.
% rows 2-7 is Joint Theta Positon q (rad).
% rows 8-13 is Joint Theta Velocity qd (rad/s).
% rows 14-19 is Joint Theta Acceleration qdd (rad/ss).
Joint7Quintic = zeros(19,traSize);
Joint7Quintic(1,:) = traLine;
%
for i = 1:DoF
    JointPathOld = zeros(TcpNumWayPoints+1,DoF);
    for j = 1:TcpNumWayPoints+1 
        JointPathOld(j,:) = JointPath(((j-1)/traSample)+1,:);
    end
    [q, qd, qdd] = quinticpolytraj([pathLine; JointPathOld(:,i)'], pathLine, traLine);
    %[q, qd, qdd] = trapveltraj([traLine; JointPath(:,i)'], traSize);
    Joint7Quintic(i+1,:) = q(2,:);
    Joint7Quintic(i+8,:) = qd(2,:);
    Joint7Quintic(i+15,:) = qdd(2,:);
    figure(i);
    plot(traLine, Joint7Quintic(i+1,:),'LineWidth',2,'DisplayName','q'); hold on;
    plot(traLine, Joint7Quintic(i+8,:),'LineWidth',2,'DisplayName','qd'); hold on;
    plot(traLine, Joint7Quintic(i+15,:),'LineWidth',2,'DisplayName','qdd'); hold off;
end
%
% Create a MATLAB file with the Joint TRAJECTORY.
ans = Joint7Quintic;
save( 'Traje7RQuintic','ans','-v7.3');
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
    TwMag = [Twist; Joint7Quintic(2:(DoF+1),i)'];
    HstR = ForwardKinematicsPOE(TwMag) * Hst0;
    TcPTra(2:7,i) = [HstR(1:3,4)' rotm2eul(HstR(1:3,1:3), 'XYZ')]';
end
% Create a MATLAB file with the TcP RESULT TRAJECTORY.
ans = TcPTra;
save( 'ToolVAL','ans','-v7.3');
%

