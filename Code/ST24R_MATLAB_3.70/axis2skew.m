%% "axis2skew" Generate a skew symmetric matrix from a vector (axis) .
% Use in SO(3).
%
% 	r = axis2skew(w)
%
% Returns a skew symmetric matrix r 3x3 from the vector 3x1 w[a1;a2;a3;].
%    |0  -a3  a2| 
% r =|a3   0 -a1|
%    |-a2 a1   0|
%
% See also: skew2axis.
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
% Revision 1.1  2020/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% r = axis2skew(w)
function r = axis2skew(w)
%
	r = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
end
%