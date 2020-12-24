%% "spavec2crm" spatial vector to cross-product MOTION (operator).
% ).
%
% 	crm = spavec2crm(spavec)
%
% Returns a matrix 6x6 from the spatial vector 6x1.
%
% It is useful to transform a cross-product operator MOTION to a matrix
% product, in the case of two 6D spatial vectors.
% "Cross = a X b" is the same as "Cross = spavec2crm(a) * b"
%
% See also: .
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
%% crm = spavec2crm(spavec)
%
function crm = spavec2crm(v)
%
crm = [    0 -v(3)  v(2)     0     0     0;
	    v(3)     0 -v(1)     0     0     0;
	   -v(2)  v(1)     0     0     0     0;
	       0 -v(6)  v(5)     0 -v(3)  v(2);
	    v(6)     0 -v(4)  v(3)     0 -v(1);
	   -v(5)  v(4)     0 -v(2)  v(1)     0];	
end
%