%% QUAT2EULZYX Convert a Quaternion to an Euler rotation ZYX axes sequence 
% Euler = Quat2EulZYX(Q) converts an angle rotation A around X axis 
% expressed as a Quaternion Q = [q0 q1 q2 q3]into the corresponding
% Euler expresion of three consecutive rototion aroung ZYX axes.
% All units in radians.
%
%   Example:
%
% See also ,
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
% along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2020/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% Euler = quat2eulZYX(Q)
%
function Euler = quat2eulZYX(Q)
%
    q0 = Q(1,1); q1 = Q(1,2); q2 = Q(1,3); q3 = Q(1,4);   
    Psi = atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2));
    Theta = asin(2*(q0*q2-q3*q1));
    Phi = atan2(2*(q0*q1+q2*q3),1-2*(q1^2+q2^2));
    Euler = [Psi Theta Phi];
%
end

