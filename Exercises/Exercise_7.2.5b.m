%% Screw Theory - EXAMPLE Trajectory Planning with Differential Kinematics.
% ABB IRB6620LX.
% Invese Differential Kinematics Algorithm with Screw Geometric Jacobian.
% Cubic Interpolation for the Joint Path Planning.
%
% The goal of this exercise is to TEST:
% TRAJECTORY PLANNING inside the workspace of the robot.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% For checking the quality of this IK solution, this exercise has 5 steps:
% STEP1: Apply ForwardKinemats for the Robot for random Mag Theta1...6
% getting a feasible set of TcP configuration (rot+tra) PATH in task-space.
% Employ some interpolation method to complete the trajectory between the
% points of the end-effector path planning, till the discretization of
% timeline. Small enough for limited changes of path planning magnitudes
% between consecutive points.
% STEP2: Evaluate the inverse DK by SCREW THEORY getting an approximation
% to the magnitud Theta1...6. Only one solution in joint-space.
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
%% E726b_ST24R_TP_ABBIRB6620LX_DK_Cubic
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
po=[0;0;0]; pu=[1.088;2.500;0]; % In fact pu has not use because Theta1=TRA
pk=[1.468;2.500;0]; pr=[2.443;2.500;0];
pf=[2.643;1.613;0]; pp=[3.000;1.613;0]; 
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [pu pk pr pf pf pf];
Joint = ['tra'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ -AxisZ -AxisZ -AxisY -AxisZ AxisX];
Twist = zeros(6,6);
for i = 1:6
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp)*rotY2tform(pi/2)*rotZ2tform(-pi/2);
%
%Motion RANGE for the robot joints POSITION rad, (by catalog).
Themax = [3 pi/180*[125 70 300 130 300]];
% Th1max is 33 but limited to 3m
Themin = [0 -pi/180*[125 180 300 130 300]];
% Th1min 1.8 but extended down to 0m by the Spatial frame definition.
%Maximum SPEED for the robot joints rad/sec, (by catalog).
%Thepmax = [3.3 pi/180*[90 90 150 120 190]];
%Thepmin = -[3.3 pi/180*[90 90 150 120 190]];
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
%Mag = [   0         0         0         0         0         0;
%     0.3464   -0.8078   -0.7780   -0.3548    0.5116   -0.7438;
%     1.4766    0.0999   -0.1260    1.4313   -0.4045   -0.6875;
%     0.5787   -0.0033   -0.9550   -0.2320   -0.8548   -1.7964;
%     0.5688    0.0547   -1.0592   -0.5110   -0.6631    0.5783;
%     0.1926   -0.6232   -0.3987    0.9909    0.0880   -1.8170;
%     1.2170   -0.5530   -0.8642   -0.7452    0.0229    0.5091;
%     0.0972   -0.5098    0.0721    0.9517    0.2652   -1.2245;
%     1.3206    0.4673    0.1181   -1.7408   -0.5735   -0.1985;
%     0.8318   -0.4947    0.2305   -1.2507   -0.1850    0.5039;
%     0.9600    0.1642   -0.7576    0.3545   -0.5373   -0.4048];
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
    % The Theta VELOCITIES values are limited by the joints spped limits.
    %Thetap = jointmag2limits(Thetap, Thepmax, Thepmin);
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
% rows 2-7 is Joint Theta Positon q (rad).
% rows 8-13 is Joint Theta Velocity qd (rad/s).
% rows 14-19 is Joint Theta Acceleration qdd (rad/ss).
Joint6Cubic = zeros(19,traSize);
Joint6Cubic(1,:) = traLine;
%
for i = 1:n
    JointPathOld = zeros(TcpNumWayPoints+1,n);
    for j = 1:TcpNumWayPoints+1 
        JointPathOld(j,:) = JointPath(((j-1)/traSample)+1,:);
    end
    [q, qd, qdd] = cubicpolytraj([pathLine; JointPathOld(:,i)'], pathLine, traLine);
    %[q, qd, qdd] = trapveltraj([traLine; JointPath(:,i)'], traSize);
    Joint6Cubic(i+1,:) = q(2,:);
    Joint6Cubic(i+7,:) = qd(2,:);
    Joint6Cubic(i+13,:) = qdd(2,:);
    figure(i);
    plot(traLine, Joint6Cubic(i+1,:),'LineWidth',2,'DisplayName','q'); hold on;
    plot(traLine, Joint6Cubic(i+7,:),'LineWidth',2,'DisplayName','qd'); hold on;
    plot(traLine, Joint6Cubic(i+13,:),'LineWidth',2,'DisplayName','qdd'); hold off;
end
%
% Create a MATLAB file with the Joint TRAJECTORY.
ans = Joint6Cubic;
save( 'Traje6Cubic','ans','-v7.3');
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
    TwMag = [Twist; Joint6Cubic(2:7,i)'];
    HstR = ForwardKinematicsPOE(TwMag) * Hst0;
    TcPTra(2:7,i) = [HstR(1:3,4)' rotm2eul(HstR(1:3,1:3), 'XYZ')]';
end
% Create a MATLAB file with the TcP RESULT TRAJECTORY.
ans = TcPTra;
save( 'ToolVAL','ans','-v7.3');
%

