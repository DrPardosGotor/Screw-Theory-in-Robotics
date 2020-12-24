%% "TFORM2TWIST" Convert a matrix "E^" 4x4 into a twist "xi" 6x1.
% Use in SE(3).
%
% 	xi = tform2twist(tform)
%
% Returns a twixt "xi" from a homogeneous "tform" matrix 4x4.
%    |v|         |W^ v|
% xi=| |  <=  E^=|    | 
%    |W|         |0  0|
%
%
% See also: twist2tform, skew2axis.
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
%% 	xi = tform2twist(tform)
function xi = tform2twist(tform)
%
	s = skew2axis([tform(1,1) tform(1,2) tform(1,3);
         tform(2,1) tform(2,2) tform(2,3);
         tform(3,1) tform(3,2) tform(3,3)]);
    xi = [tform(1,4);tform(2,4);tform(3,4);s(1,1);s(2,1);s(3,1)];
end
%