%% "skew2axis" Generate an axis from an skew symmetric matrix.
% Use in SO(3).
%
% 	w = skew2axis(r)
%
% Returns a vector 3x1 w[a1;a2;a3;] from a skew symmetric matrix r 3x3 .
%    |0  -a3  a2| 
% r =|a3   0 -a1|
%    |-a2 a1   0|
%
% See also: axis2skew.
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
%% w = skew2axis(r)
function w = skew2axis(r)
%
	w = [r(3,2); r(1,3); r(2,1)];
end
%