%% MINUSPOSEEUL - Pose difference between two trajectoria targets
% the poses are expresed as Euler coordinates with position X-Y-Z and 
% orientation with scheme X-Y-Z: Pose = [trvX trvY trvZ rotX rotY rotZ]
%
% 	posediff = minusposeEul(pose1, pose2)
%
% Returns a pose difference expresed as Euler coordinates with the same
% structure as the inputs: pose difference = pose2 - pose1
% expressed in the same reference frame as both input poses.
% The formulation is as follows:
% Position difference = [trvX1-trvX2 trvY1-trvY2 trvZ1-trvZ2]
% Rotation difference = R12 = R01' * R02
% Pass R12 to axis/angle form, plus multiply the axis vector by the angle,
% getting the difference as an Euler X-Y-Z vector in the pose1 frame.
% The rotation difference is then expressed on the spatial common frame
% by multiplying the euler vector by the pose1 (R01) transformation.
%
% See also: 
%
% Copyright (C) 2001-2020, by Dr. Jose M. Pardos-Gotor.
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
% Revision 1.1  2019/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% 	posediff = minusposeEul(pose1, pose2)
%
function posediff = minusposeEul(pose1, pose2)
%  
    vtcpS = (pose2(1:3) - pose1(1:3));
    R01 = eul2rotm(pose1(4:6),'XYZ');
    R12axang = rotm2axang(R01' * eul2rotm(pose2(4:6),'XYZ'));
    wstS = R01 * (R12axang(1:3) * R12axang(4))';
    posediff = [vtcpS wstS']';
%
end
%
