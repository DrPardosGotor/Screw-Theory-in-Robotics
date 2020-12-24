%% "TWIST2JOINT" Find the JOINT: AXIS, a POINT on axis, and type of joint.
% Use in SE(3).
%
% Where AXIS (3x1) & POINT (3x1) on that axis & JOINTYPE ('rot' or 'tra')
% From the TWIST "xi" (6x1) has the two components v (3x1) and w (3x1)
% for the function to work with both ROTATION & TRANSLATION movements.
%
%   (Axis, Point, JointType) = twist2joint(xi)
%
%
%     |v|          W x v
% xi =| | => l = --------- + Kw : K in R, if w is not ZERO.
%     |w|   .     ||w||^2
%     |v|          
% xi =| | => l = 0 + Kv : K in R, if w = 0.
%     |w|   .     
%
% Be careful, because this definition is an extension for defining a SCREW 
% associated with a twist, but It does not mean that the twist is a screw
% with the translation component parallel to the rotation component.
%
% See also: twistpitch, twistmagnitude.
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
%% 	[Axis, Point, JointType] = twist2joint(xi)
function [Axis, Point, JointType] = twist2joint(xi)
    v = xi(1:3);
    w = xi(4:6);
    if norm(w) == 0
        Axis = v / norm(v);
        Point = [0; 0; 0]';
        JointType = 'tra';
    else
        Axis = w / norm(w);
        Point = cross(w,v)/(norm(w)^2);     
        JointType = 'rot';
    end
end
%
