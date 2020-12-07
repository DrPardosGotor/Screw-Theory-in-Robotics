%% Screw Theory - EXAMPLE Trajectory Planning with Differential Kinematics.
% ABB IRB1600.
% Invese Differential Kinematics Algorithm with Screw Geometric Jacobian.
% Quintic Interpolation for the Joint Path Planning.
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
%% E724c_ST24R_TP_ABBIRB1600_DK_Quintic
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
% Motion RANGE for the robot joints POSITION rad, (by catalog).
Themax = pi/180*[165 110 70 160 120 400];
Themin = -pi/180*[165 110 110 160 120 400];
% Maximum VELOCITY for the robot joints rad/sec, (by catalog).
Thepmax = pi/180*[250 250 250 320 320 420];
Thepmin = -pi/180*[250 250 250 320 320 420];
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
Joint6Quintic = zeros(19,traSize);
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
    Joint6Quintic(i+7,:) = qd(2,:);
    Joint6Quintic(i+13,:) = qdd(2,:);
    figure(i);
    plot(traLine, Joint6Quintic(i+1,:),'LineWidth',2,'DisplayName','q'); hold on;
    plot(traLine, Joint6Quintic(i+7,:),'LineWidth',2,'DisplayName','qd'); hold on;
    plot(traLine, Joint6Quintic(i+13,:),'LineWidth',2,'DisplayName','qdd'); hold off;
end
%
% Create a MATLAB file with the Joint TRAJECTORY.
ans = Joint6Quintic;
save( 'Traje6RQuintic','ans','-v7.3');
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
    TwMag = [Twist; Joint6Quintic(2:7,i)'];
    HstR = ForwardKinematicsPOE(TwMag) * Hst0;
    TcPTra(2:7,i) = [HstR(1:3,4)' rotm2eul(HstR(1:3,1:3), 'XYZ')]';
end
% Create a MATLAB file with the TcP RESULT TRAJECTORY.
ans = TcPTra;
save( 'ToolVAL','ans','-v7.3');
%

