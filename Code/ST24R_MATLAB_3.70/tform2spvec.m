%% tform2spvec Convert an homogeneous transformation to 6D Spatial Vector  
% Spatial Vector = tform2spvec(H) converts an homegeneous transformation
% matrix (4x4) to the corresponding Spatial Vector (6x1) expressed as:
% spvec = [rx ry rz tx ty tz]'
% where rx, ry and rz are the rotations on the axis X-Y-Z, but not intended
% as consecutive Euler rotations, but as the unique expression of rotation
% around a singe axis and then normarlized over the XYZ frame.
% wher tx, ty and tz are the translation of the motion.
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
%% spvec = tform2spvec(H)
%
function spvec = tform2spvec(H)
%
    spvec = [0; 0; 0; H(1:3,4)];
    axang2spvec = rotm2axang(H(1:3,1:3));
    spvec(1:3) = axang2spvec(1:3)*axang2spvec(4);
%
end

