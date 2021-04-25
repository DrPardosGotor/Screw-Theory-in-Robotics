%% Function "F110_CreateToolTra_Trig"
% 3D Sin target for Tool trajectory generation.
%
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% function TrajOut = F110_CreateToolTra_Sin(Toolmin, Toolmax, tsample, tratime)
%
% INPUTS:
% Toolmin (1x6) minimum values for the Tool Pos & Rot dimensions.
% Toolmax (1x6) maximum values for the Tool Pos & Rot dimensions.
% tratime (1x1) sec time duration for the close 3D trajectory.
% timesample (1x1) sec sample time for each point of the trejectory
% 
% OUTPUTS:
% it is a timeseries mat file 'TrajOut' (7xn) with the trajectory for
% the TOOL by rows:
% t(1,:) = time line sec.
% TcPPosx(2,:) = Tool Center Point x position.
% TcPPosy(3,:) = Tool Center Point y position.
% TcPPosz(4,:) = Tool Center Point z position.
% TcPRotx(5,:) = Tool Center Point x rotation.
% TcPRoty(6,:) = Tool Center Point y rotation.
% TcPRotz(7,:) = Tool Center Point z rotation.
%
% Use case:
% F110_CreateToolTra_Trig([0.2 0.2 0.2 0 pi 0], [0.3 0.4 0.5 pi/2 pi+pi/2 pi/2], 0.001, 10)
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
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% F110_CreateToolTra_Trig
%
function TrajOut = F110_CreateToolTra_Trig(Toolmin, Toolmax, tsample, tratime) % #codegen
%
% Reading the TOOL trajectory from the inputs.
% One second is added to timeline for the robot to reach the initial point
timeline = (0:tsample:tratime);
trasize = size(timeline,2);
TrajOut = zeros(7,trasize);
%
TrajOut(1,:) = timeline;
%
TrajOutAmp01 = (Toolmax(1)-Toolmin(1))/2;
TrajOutLev01 = (Toolmax(1)+Toolmin(1))/2;
TrajOutAmp02 = (Toolmax(2)-Toolmin(2))/2;
TrajOutLev02 = (Toolmax(2)+Toolmin(2))/2;
TrajOutAmp03 = (Toolmax(3)-Toolmin(3))/2;
TrajOutLev03 = (Toolmax(3)+Toolmin(3))/2;
TrajOutAmp04 = (Toolmax(4)-Toolmin(4))/2;
TrajOutLev04 = (Toolmax(4)+Toolmin(4))/2;
TrajOutAmp05 = (Toolmax(5)-Toolmin(5))/2;
TrajOutLev05 = (Toolmax(5)+Toolmin(5))/2;
TrajOutAmp06 = (Toolmax(6)-Toolmin(6))/2;
TrajOutLev06 = (Toolmax(6)+Toolmin(6))/2;
TrajOut(2,:) = TrajOutAmp01*cos(2*pi/tratime*TrajOut(1,:))+TrajOutLev01;
TrajOut(3,:) = TrajOutAmp02*cos(2*pi/tratime*TrajOut(1,:))+TrajOutLev02;
TrajOut(4,:) = TrajOutAmp03*sin(2*pi/tratime*TrajOut(1,:))+TrajOutLev03;
TrajOut(5,:) = TrajOutAmp04*cos(2*pi/tratime*TrajOut(1,:))+TrajOutLev04;
%TrajOut(5,:) = pi/2;
TrajOut(6,:) = TrajOutAmp05*cos(2*pi/tratime*TrajOut(1,:))+TrajOutLev05;
%TrajOut(6,:) = 0;
TrajOut(7,:) = TrajOutAmp06*sin(2*pi/tratime*TrajOut(1,:))+TrajOutLev06;
%
% TrajOut = timeseries(TrajOut(2:8,:),timeline);
save('Traj_CARTE_Pos&Rot', 'TrajOut','-v7.3');
%
end
%