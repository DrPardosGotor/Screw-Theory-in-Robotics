%% Screw Theory in Robotics
% An Illustrated and Practicable Introduction to Modern Mechanics
% by CRC Press
% © 2022 Jose M Pardos-Gotor
%
%% Ch3 - FORWARD KINEMATICS.
%
% Exercise 3.2.3: Puma Robots (e.g., ABB IRB120)
%
% Denavit-Hartenberg Algorithm.
% Calculate the Homogeneous Matrix transformation for the end-effector of
% a ABB IRB120 (ToolDown configuration) PUMA type robot with six Joints.
%
%
% Using Screw Theory Functions from ST24R.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Copyright (C) 2001-2021, by Dr. Jose M. Pardos-Gotor.
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
% You should have received a copy of the GNU Leser General Public License
% along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2021/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% MATLAB Code
%
clear
clc
%
% Random values for the Joint (Theta) magnitudes
The = zeros(1,6);
for i = 1:6
    The(i) = (rand-rand)*pi; % Magnitudes Theta1-Theta6 for testing.
end
%
% Denavith-Hartenber Parameters
dhparams = [0.29 The(1) 0 -pi/2; 0 The(2)-pi/2 0.27 0];
dhparams = [dhparams; 0 The(3) 0.07 -pi/2; 0.302 The(4) 0 pi/2];
dhparams = [dhparams; 0 The(5)-pi/2 0 pi/2; 0.16 The(6) 0 0];
%
%
tic;
Hst = ForwardKinematicsDHD(dhparams)
tFKDHD = round(toc*1000,3);
timeFKDHD= ['Time to solve FK D-H Direct ', num2str(tFKDHD),' ms']
%
tic;
Hst1 = ForwardKinematicsDHC(dhparams)
tFKDH = round(toc*1000,3);
timeFKDH= ['Time to solve FK D-H ST24R ', num2str(tFKDH),' ms']
%
%