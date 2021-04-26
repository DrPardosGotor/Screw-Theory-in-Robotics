%% Function "F110_CreateToolTra_SPL"
% 3D spline target for Tool trajectory generation.
%
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% function TrajOut = F110_CreateToolTra_SPL(XYZmin, XYZmax, tsample, tratime)
%
% INPUTS:
% XYZmin (1x3) minimum values for the XYZ 3D dimensions.
% XYZmax (1x3) maximum values for the XYZ 3D dimensions.
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
% F110_CreateToolTra_SPL([0.5 -0.25 0.5], [0.6 0.25 0.75], 0.001, 10)
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
% Revision 1.1  2019/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% F110_CreateToolTra_SPL
%
function TrajOut = F110_CreateToolTra_SPL(XYZmin, XYZmax, tsample, tratime) % #codegen
%
% Reading the TOOL trajectory from the inputs.
% One second is added to timeline for the robot to reach the initial point
timeline = (0:tsample:tratime);
trasize = size(timeline,2);
TrajOut = zeros(7,trasize);
%
onesize = size((0:tsample:1),2);
traquar = round((trasize-1)/4);
%
TrajOut(1,:) = timeline;
%
TrajOut(2,1:onesize) = (XYZmax(1)+XYZmin(1))/2;
TrajOut(2,trasize) = TrajOut(2,onesize);
TrajOut(2,onesize+traquar) = XYZmin(1);
TrajOut(2,onesize+2*traquar) = TrajOut(2,onesize);
TrajOut(2,onesize+3*traquar) = XYZmax(1);
TraX = [TrajOut(1,onesize), TrajOut(1,onesize+traquar), TrajOut(1,onesize+2*traquar)];
TraX = [TraX, TrajOut(1,onesize+3*traquar), TrajOut(1,trasize)];
TraY = [TrajOut(2,onesize), TrajOut(2,onesize+traquar), TrajOut(2,onesize+2*traquar)];
TraY = [TraY, TrajOut(2,onesize+3*traquar), TrajOut(2,trasize)];
TrajOut(2,onesize:trasize) = spline(TraX,TraY,TrajOut(1,onesize:trasize));
%
TrajOut(3,1:onesize) = XYZmax(2);
TrajOut(3,trasize) = TrajOut(3,onesize);
TrajOut(3,onesize+traquar) = (XYZmax(2)+XYZmin(2))/2;
TrajOut(3,onesize+2*traquar) = XYZmin(2);
TrajOut(3,onesize+3*traquar) = TrajOut(3,onesize+traquar);
TraY = [TrajOut(3,onesize), TrajOut(3,onesize+traquar), TrajOut(3,onesize+2*traquar)];
TraY = [TraY, TrajOut(3,onesize+3*traquar), TrajOut(3,trasize)];
TrajOut(3,onesize:trasize) = spline(TraX,TraY,TrajOut(1,onesize:trasize));
%
TrajOut(4,1:onesize) = (XYZmax(3)+XYZmin(3))/2;
TrajOut(4,trasize) = TrajOut(4,onesize);
TrajOut(4,onesize+traquar) = XYZmax(3);
TrajOut(4,onesize+2*traquar) = TrajOut(4,onesize);
TrajOut(4,onesize+3*traquar) = XYZmin(3);
TraY = [TrajOut(4,onesize), TrajOut(4,onesize+traquar), TrajOut(4,onesize+2*traquar)];
TraY = [TraY, TrajOut(4,onesize+3*traquar), TrajOut(4,trasize)];
TrajOut(4,onesize:trasize) = spline(TraX,TraY,TrajOut(1,onesize:trasize));
%
TrajOut(5,1:onesize) = pi/2;
TrajOut(5,trasize) = TrajOut(5,onesize);
TrajOut(5,onesize+traquar) = 5*pi/8;
TrajOut(5,onesize+2*traquar) = pi/2;
TrajOut(5,onesize+3*traquar) = 3*pi/8;
TraY = [TrajOut(5,onesize), TrajOut(5,onesize+traquar), TrajOut(5,onesize+2*traquar)];
TraY = [TraY, TrajOut(5,onesize+3*traquar), TrajOut(5,trasize)];
TrajOut(5,onesize:trasize) = spline(TraX,TraY,TrajOut(1,onesize:trasize));
%
TrajOut(6,1:onesize) = pi/2;
TrajOut(6,trasize) = TrajOut(6,onesize);
TrajOut(6,onesize+traquar) = 5*pi/8;
TrajOut(6,onesize+2*traquar) = pi/2;
TrajOut(6,onesize+3*traquar) = 3*pi/8;
TraY = [TrajOut(6,onesize), TrajOut(6,onesize+traquar), TrajOut(6,onesize+2*traquar)];
TraY = [TraY, TrajOut(6,onesize+3*traquar), TrajOut(6,trasize)];
TrajOut(6,onesize:trasize) = spline(TraX,TraY,TrajOut(1,onesize:trasize));
%
TrajOut(7,1:onesize) = 90*pi/180;
TrajOut(7,trasize) = TrajOut(7,onesize);
TrajOut(7,onesize+traquar) = 100*pi/180;
TrajOut(7,onesize+2*traquar) = 90*pi/180;
TrajOut(7,onesize+3*traquar) = 80*pi/180;
TraY = [TrajOut(7,onesize), TrajOut(7,onesize+traquar), TrajOut(7,onesize+2*traquar)];
TraY = [TraY, TrajOut(7,onesize+3*traquar), TrajOut(7,trasize)];
TrajOut(7,onesize:trasize) = spline(TraX,TraY,TrajOut(1,onesize:trasize));
%
% TrajOut = timeseries(TrajOut(2:8,:),timeline);
save('Traj_CARTE_Pos&Rot', 'TrajOut','-v7.3');
%
end
%