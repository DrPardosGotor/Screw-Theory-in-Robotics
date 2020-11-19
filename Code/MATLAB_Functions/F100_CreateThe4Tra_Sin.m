%% Function "F100_CreateThe4Tra_Sin"
% Generation of Trajectories for FOUR Joints (shape SIN).
% Trajectories intended for Forward Kinematics Exercises and Simulations
%
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% function TrajOut = F100_CreateThe4Tra_Sin(Themax, Thmin, tsample, tratime)
%
% INPUTS:
% Themax (1x4) rad maximum values for the Joint magnitudes.
% Themin (1x4) rad minimum values for the Joint magnitudes.
% tratime (1x1) sec time duration for the trajectory.
% timesample (1x1) sec sample time for each point of the trejectory
% 
% OUTPUTS:
% it is a timeseries mat file 'TrajOut' (5xn) with the trajectory for
% the JOINTS by rows:
% t(1,:) = time line sec.
% TrajOut(2,:) = Joint1 Theta position (rad).
% TrajOut(3,:) = Joint2 Theta position (rad).
% TrajOut(4,:) = Joint3 Theta position (rad).
%
% Use case:
% Thmax = pi/180*[360 360 360 360]/3;
% Thmin = pi/180*[360 360 360 360]/3;
% F100_CreateThe4Tra_Sin(Thmax, Thmin, 0.001, 10);
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
% You should have received a copy of the GNU Leser General Public License
% along with ST24R. If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2020/02/11 00:00:01
% General cleanup of code: help comments, sF17_IIWA_ThetaTraje_Sinee also, copyright
% references, clarification of functions.
%
%% F100_CreateThe4Tra_Sin
%
function TrajOut = F100_CreateThe4Tra_Sin(Themax, Themin, tsample, tratime) % #codegen
%
% One second is added to timeline for the robot to reach the initial point
timeline = (0:tsample:tratime);
trasize = size(timeline,2);
TrajOut = zeros(5,trasize);
%
TrajOut(1,:) = timeline;
%
TrajOutAmp01 = (abs(Themax(1))+abs(Themin(1)))/2;
TrajOutLev01 = (Themax(1)+Themin(1))/2;
TrajOutAmp02 = (abs(Themax(2))+abs(Themin(2)))/2;
TrajOutLev02 = (Themax(2)+Themin(2))/2;
TrajOutAmp03 = (abs(Themax(3))+abs(Themin(3)))/2;
TrajOutLev03 = (Themax(3)+Themin(3))/2;
TrajOutAmp04 = (abs(Themax(4))+abs(Themin(4)))/2;
TrajOutLev04 = (Themax(4)+Themin(4))/2;
TrajOut(2,:) = TrajOutAmp01*sin(2*pi/tratime*TrajOut(1,:))+TrajOutLev01;
TrajOut(3,:) = TrajOutAmp02*sin(2*pi/tratime*TrajOut(1,:))+TrajOutLev02;
TrajOut(4,:) = TrajOutAmp03*sin(2*pi/tratime*TrajOut(1,:))+TrajOutLev03;
TrajOut(5,:) = TrajOutAmp04*sin(2*pi/tratime*TrajOut(1,:))+TrajOutLev04;
%
save('Theta4Traj_Sin', 'TrajOut','-v7.3');
%
end
%