%% Screw Theory - EXAMPLE Trajectory Planning with Differential Kinematics.
% ABB IRB910SC.
% Invese Differential Kinematics Algorithm with Screw Geometric Jacobian.
% Quintic Interpolation for the Joint Path Planning.
%
% The goal of this exercise is to TEST:
% TRAJECTORY PLANNING inside the workspace of the robot.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% For checking the quality of this IK solution, this exercise has 5 steps:
% STEP1: Apply ForwardKinemats for the Robot for random Mag Theta1...4
% getting a feasible set of TcP configuration (rot+tra) PATH in task-space.
% Employ some interpolation method to complete the trajectory between the
% points of the end-effector path planning, till the discretization of
% timeline. Small enough for limited changes of path planning magnitudes
% between consecutive points.
% STEP2: Evaluate the inverse DK by SCREW THEORY getting an approximation
% to the magnitud Theta1...4. Only one solution in joint-space.
% the INVERSE (Joint Thetap Velocities) DK, based on Tool Velocities.
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
%% E728c_ST24R_TP_ABBIRB910SC_DK_Quintic
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
%Motion RANGE for the robot joints POSITION rad, (by catalog).
Themax = [pi/180*140 pi/180*150 0 pi/180*400];
Themin = [-pi/180*140 -pi/180*150 -0.125 -pi/180*400];
% Maximum SPEED for the robot joints m/s and rad/sec, (by catalog).
Thepmax = [7.58 7.58 1.02 pi/180*2400];
Thepmin = -[7.58 7.58 1.02 pi/180*2400];
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
    Mag(i,1) = 1/5*(rand*Themax(1)+rand*Themin(1));
    Mag(i,2) = 1/2*(rand*Themax(2)+rand*Themin(2));
    Mag(i,3) = rand*Themax(3)+rand*Themin(3);
    Mag(i,4) = 1/50*(rand*Themax(4)+rand*Themin(4));
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
JointPath = zeros(traSize,n);
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
    Thetap = (pinv(JstS)*VstS)'; % it is giving worse results.
    % This formulation is faster, but only works for Square matrices 
    %Thetap = (JstS\VstS)';
    % Using Moore-Penrose generalized inverse for getting Theta velocities.
    %Thetap = ((pinv(JstS'*JstS) * JstS')*VstS)';
    % The Theta VELOCITIES values are limited by the joints spped limits.
    Thetap = jointmag2limits(Thetap, Thepmax, Thepmin);
    %
    % from Inverse DK we get the incremental joint coordinates
    % and then integrating with EULER Explicit Method the Theta VALUE 
    % with the new joint positions vector OUTPUT.
    % consider the integration step size.
    Theta = JointPath(i,:) + (Thetap * traSample);
    % The Theta POSITION values are limited by the joints position limits.
    %Theta = jointmag2limits(Theta, Themax, Themin);
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
% rows 2-5 is Joint Theta Positon q (rad).
% rows 6-9 is Joint Theta Velocity qd (rad/s).
% rows 10-13 is Joint Theta Acceleration qdd (rad/ss).
Joint6Quintic = zeros(13,traSize);
Joint6Quintic(1,:) = traLine;
%
for i = 1:n
    JointPathOld = zeros(TcpNumWayPoints+1,n);
    for j = 1:TcpNumWayPoints+1 
        JointPathOld(j,:) = JointPath(((j-1)/traSample)+1,:);
    end
    [q, qd, qdd] = quinticpolytraj([pathLine; JointPathOld(:,i)'], pathLine, traLine);
    %[q, qd, qdd] = trapveltraj([traLine; JointPath(:,i)'], traSize);
    Joint6Quintic(i+1,:) = q(2,:);
    Joint6Quintic(i+5,:) = qd(2,:);
    Joint6Quintic(i+9,:) = qdd(2,:);
    figure(i);
    plot(traLine, Joint6Quintic(i+1,:),'LineWidth',2,'DisplayName','q'); hold on;
    plot(traLine, Joint6Quintic(i+5,:),'LineWidth',2,'DisplayName','qd'); hold on;
    plot(traLine, Joint6Quintic(i+9,:),'LineWidth',2,'DisplayName','qdd'); hold off;
end
%
% Create a MATLAB file with the Joint TRAJECTORY.
ans = Joint6Quintic;
save( 'Traje4Quintic','ans','-v7.3');
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
    TwMag = [Twist; Joint6Quintic(2:5,i)'];
    HstR = ForwardKinematicsPOE(TwMag) * Hst0;
    TcPTra(2:7,i) = [HstR(1:3,4)' rotm2eul(HstR(1:3,1:3), 'XYZ')]';
end
% Create a MATLAB file with the TcP RESULT TRAJECTORY.
ans = TcPTra;
save( 'ToolVAL','ans','-v7.3');
%

