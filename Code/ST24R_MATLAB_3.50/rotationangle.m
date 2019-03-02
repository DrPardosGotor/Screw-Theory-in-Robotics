%% "rotationangle" Find the angle "Theta" of a rotation matrix.
% Use in SO(3).
%
% 	t = rotationangle(r)
%
% Find the angle "Theta" of a rotation matrix. R is a Rotation matrix 3x3. 
%             
% t = arccos[(Trace(r)-1)/2]           
% 
% See also: motiontwist, rotationaxis.
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
%% th = rotationangle(r) 
function t = rotationangle(r)
%  
    tr = ((r(1,1)+r(2,2)+r(3,3))-1)/2;
    if tr <-1
  	    tr = -1;
    elseif tr > 1
        tr = 1;
    end
    t = acos(tr);
end
%
    