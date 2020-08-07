%% Screw Theory for Robotics - FORWARD Kinematics.
% PUMA robot - ABB IRB 120 - Tool Down.
% Screw Theory POE.
%
% Calculate the Homogeneous Matrix transformation for the end-effector of
% a ABB IRB120 (ToolDown configuration) type robot of six Joints
%
% Using Screw Theory Functions from ST24R.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
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
%% E331_ST24R_FK_ABBIRB120_POE
%
clear
clc
%
% Mechanical characteristics of the Robot:
po=[0;0;0]; pk=[0; 0; 0.290]; pr=[0; 0; 0.560];
pf=[0.302; 0; 0.630]; pp=[0.302; 0; 0.470];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pk pr pf pf pf];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ AxisY AxisY AxisX AxisY -AxisZ];
for i = 1:6
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp)*rotY2tform(pi);
%
The = zeros(1,6);
for i = 1:6
    The(i) = (rand-rand)*pi; % for testing various Theta1-Theta6
end
%
%
tic;
TwMag = [Twist; The]; % assign the rand values to joint Theta magnitudes.
HstR = ForwardKinematicsPOE(TwMag);
noap = HstR * Hst0
tFKST = round(toc*1000,3);
timeFKST= ['Time to solve FK D-H ST24R ', num2str(tFKST),' ms']
%
%