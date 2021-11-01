%% Screw Theory in Robotics
% An Illustrated and Practicable Introduction to Modern Mechanics
% by CRC Press
% © 2022 Jose M Pardos-Gotor
%
%% Ch2 - MATHEMATICAL TOOLS
% 
% Exercise 2.3.6: Exponential Rotation 
%
% Transform a vector rt(3;2;1) expressed in coordinates of the mobil system
% T(OUVW), to its expression rs? in coordinates of the reference system
% S(OXYZ). The system T(OUVW) is rotated by an angle gamma(pi/4) with
% respect to the axis OZ of the system S(OXYZ).
%
% Using Screw Theory Functions from ST24R.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
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
%% MATLAB Code
%
clear;
clc;
gamma = pi/4;
W = [0 0 1];
rotm = expAxAng([W gamma]); % expAxAng implements Rodrigues' formula.
rt = [3; 2; 1];
rs = rotm * rt