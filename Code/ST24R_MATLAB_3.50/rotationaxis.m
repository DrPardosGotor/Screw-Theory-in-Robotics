%% "rotationaxis" Find the axis of a rotation matrix 
% Use in SO(3).
%
% 	W = rotationaxis(R, THETA)
%
% Find the axis of a rotation matrix. R is the Rotation matrix 3x3. 
%                |r32-r23|
% W = 1/(2*sinTh)|r13-r31| 
%                |r21-r12|
% If sinTh=0 the result is the null space.
%
% See also: motiontwist, rotationangle.
%
% Copyright (C) 2001-2019, by Dr. Jose M. Pardos-Gotor.
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
%% w = rotationaxis(r, t)
function w = rotationaxis(r, t)
%  
    st = sin(t);
    if st == 0
        w = [0;0;0];
    else
        w = 1/(2*st)*[r(3,2)-r(2,3);r(1,3)-r(3,1);r(2,1)-r(1,2)];
    end
end
%
    